#include "communicate_task.h"
#include "cmsis_os.h"
#include "protocol_lite.h"
#include "bsp_usb.h"
#include "linux_list.h"
#include "log.h"
#include "os_timer.h"
#include "offline_service.h"
#include "bsp_led.h"


// 事件 每当回调函数调用完成后会发送 事件标志为cmd_id
extern osEventFlagsId_t cmdReceivedEventHandle;

// 回调函数列表节点
typedef struct _ProtocolCallbackListNode
{
    short life;                         // -1表示永久 每次调用会减1 当减为0时会销毁
    void* parameter;                    // 传入callback的参数
    uint16_t cmd_id;                    // 待接收的指令
	protocol_cmd_callback_t callback;   // 回调函数
    list_t list;                        // 列表
} ProtocolCallbackListNode;

// 回调函数列表
LIST_HEAD(protocol_callback_list);

// 注册回调函数
void __register_cmd_callback(uint16_t cmd_id, protocol_cmd_callback_t callback, short life, void* parameter, const char *callback_name)
{
    var_cpu_sr();
    
    ProtocolCallbackListNode* node = (ProtocolCallbackListNode*)pvPortMalloc(sizeof(ProtocolCallbackListNode));
    if(node == NULL)
    {
        log_printf("Protocol callback registered failure, cmd_id: 0x%04x, callback: %s().", cmd_id, callback_name);
    }
    node->life = life;
    node->parameter = parameter;
    node->cmd_id = cmd_id;
    node->callback = callback;
    
    enter_critical();
    list_add_tail(&node->list, &protocol_callback_list);
    exit_critical();
    
    if(life < 0)
        log_printf("Protocol callback is registered, cmd_id: 0x%04x, callback: %s().", cmd_id, callback_name);
    else
        log_printf("Protocol callback is registered, cmd_id: 0x%04x, callback: %s(), life: %d.", cmd_id, callback_name, life);
}

// 删除回调函数
void remove_cmd_callback(uint16_t cmd_id)
{
    var_cpu_sr();
    
    list_t* pos;
    list_t* temp;
    
    enter_critical();
    list_for_each_safe(pos, temp, &protocol_callback_list)
    {
        ProtocolCallbackListNode* node = list_entry(pos, ProtocolCallbackListNode, list);

        if(node->cmd_id == cmd_id)
        {
            list_del(pos);
            vPortFree(node);
        }
    }
    exit_critical();
    
}

// 发送指令和数据
int send_cmd_with_data(uint16_t cmd_id, uint8_t *p_data, uint16_t len)
{
    static uint8_t frame_buffer[PROTOCOL_FRAME_MAX_SIZE];
    
    if(len > PROTOCOL_DATA_MAX_SIZE)
        len = PROTOCOL_DATA_MAX_SIZE;
    
    uint32_t frame_size = protocol_pack_data_to_buffer(cmd_id, p_data, len, frame_buffer);
    
    usb_interface_send(frame_buffer, frame_size);
    
    return len;
}

// RPC远程调用回调函数 用于实现RPC
PROTOCOL_CALLBACK_FUNCTION(__remote_procedure_call_callback)
{
    // 将返回的数据进行一次拷贝
    RPCResponse *response = parameter;
    memcpy(response->data, p_data, len);
    response->len = len;
}

// RPC远程调用 结果将保存至p_response_data 返回值1表示调用成功 timeout单位为ms
int remote_procedure_call(  uint16_t request_cmd_id, uint8_t *p_request_data, uint16_t request_len,
                            uint16_t response_cmd_id, RPCResponse *p_rpc_response, uint32_t timeout)
{
    uint32_t start_time = get_time_ms();
    
    // 注册临时回调函数
    register_cmd_callback(response_cmd_id, __remote_procedure_call_callback, 1, p_rpc_response);
    
    // 发送请求
    osEventFlagsClear(cmdReceivedEventHandle, 0xffffffffu);
    send_cmd_with_data(request_cmd_id, p_request_data, request_len);
    
    // 等待结果
    uint32_t duration;
    while(1)
    {
        duration = get_time_ms() - start_time;
        if(duration > timeout)
        {
            break;
        }
        
        uint32_t result = osEventFlagsWait(cmdReceivedEventHandle, response_cmd_id, osFlagsWaitAll, timeout - duration);
        if(result == response_cmd_id)
        {
            return 1;
        }
    }
    
    remove_cmd_callback(response_cmd_id);
    return 0;
}

// 执行回调函数
void execute_cmd_callback(uint16_t cmd_id, uint8_t *p_data, uint16_t len)
{
    var_cpu_sr();
    
    list_t* pos;
    list_t* temp;
    
    enter_critical();
    list_for_each_safe(pos, temp, &protocol_callback_list)
    {
        ProtocolCallbackListNode* node = list_entry(pos, ProtocolCallbackListNode, list);
        
        // 当剩余次数为0时删除节点
        if(node->life == 0)
        {
            list_del(pos);
            vPortFree(node);
            continue;
        }

        if(node->cmd_id == cmd_id)
        {
            // 调用回调函数
            node->callback(cmd_id, p_data, len, node->parameter);
            
            // 回调函数执行完成 发送对应事件
            if(node->life > 0)
            {
                osEventFlagsSet(cmdReceivedEventHandle, cmd_id);
            }
            
            // 扣除剩余次数 忽略永久回调(-1)
            if(node->life > 0)
            {
                node->life--;
            }
        }
    }
    exit_critical();
}

// 发送心跳
int32_t heart_timer(void *argc)
{
    send_cmd_with_data(CMD_HEART, NULL, 0);
    return 0;
}

// 离线服务
int communicate_offline_id = -1;
void communicate_offline_callback(void)
{
    log_printf("SDK Offline.");
    led_red_on();
}
void communicate_online_callback(void)
{
    log_printf("SDK Online.");
    led_red_off();
}
PROTOCOL_CALLBACK_FUNCTION(cmd_heart_callback)
{
    if(communicate_offline_id > -1)
    {
        offline_hook(communicate_offline_id);
    }
}

// 通讯任务
void communicate_task(void *argument)
{
    // 注册协议回调函数
    extern void register_all_callbacks(void);
    register_all_callbacks();
    
    // 注册心跳定时器
    soft_timer_register(usb_tx_flush, NULL, 1);
    soft_timer_register(heart_timer, NULL, 500);
    
    // 注册离线服务
    register_cmd_callback(CMD_HEART, cmd_heart_callback, -1, NULL);
    while(1)
    {
        communicate_offline_id = register_offline_callback(NULL, communicate_online_callback, NULL, communicate_offline_callback, 1000);
        if(communicate_offline_id > -1)
            break;
        osDelay(1);
    }
    
    // 解包对象
    unpack_data_t unpack_data_obj;
    protocol_initialize_unpack_object(&unpack_data_obj);
    
    while(1)
    {
        
        while(1)
        {
            int ch = usb_rx_fifo_read();
            
            if (ch >= 0)
            {
                // 解析数据
                if(protocol_unpack_byte(&unpack_data_obj, ch))
                {
                    // 成功解析后调用对应的回调函数
                    execute_cmd_callback(unpack_data_obj.cmd_id, unpack_data_obj.data, unpack_data_obj.data_len);
                }
            }
            else
            {
                break;
            }
            
        }
        
        osDelay(1);
    }
}
