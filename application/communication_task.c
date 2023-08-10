#include "communication_task.h"
#include <cmsis_os.h>
#include "mem_mang.h"
#include "ahrs.h"
#include "event_mgr.h"
#include "event.h"
#include "os_timer.h"
#include "app_manage.h"

#define LOG_TAG "communication"
#define LOG_OUTPUT_LEVEL 4
#include "log.h"

#define PROTOCOL_MAX_FRAME_LENGTH (1000)
static protocol_stream_t unpack_stream_object;
osThreadId communication_task_t;

typedef struct
{
    uint16_t cmd_id;
	protocol_cmd_callback_t callback;   // 回调函数
    list_t list;                        // 列表
} ProtocolCallbackListNode;

LIST_HEAD(protocol_callback_list);

void __register_cmd_callback(uint16_t cmd_id, protocol_cmd_callback_t callback, const char *callback_name)
{
    var_cpu_sr();
    
    ProtocolCallbackListNode* node = (ProtocolCallbackListNode*)pvPortMalloc(sizeof(ProtocolCallbackListNode));
    if(node == NULL)
    {
        log_i("Protocol callback registered failure, cmd_id: 0x%04x, callback: %s().", cmd_id, callback_name);
    }
    node->cmd_id = cmd_id;
    node->callback = callback;
    
    enter_critical();
    list_add_tail(&node->list, &protocol_callback_list);
    exit_critical();
    
    log_i("Protocol callback is registered, cmd_id: 0x%04x, callback: %s().", cmd_id, callback_name);
}

void dispatch_frame(uint16_t cmd_id, uint8_t *p_data, uint16_t len)
{
    list_t* pos;
    
    list_for_each(pos, &protocol_callback_list)
    {
        ProtocolCallbackListNode* node = list_entry(pos, ProtocolCallbackListNode, list);
        
        if(node->cmd_id == cmd_id)
        {
            node->callback(cmd_id, p_data, len);
        }
    }
}

PROTOCOL_CALLBACK_FUNCTION(sync_machine_time)
{
    // Set timestamp
    cmd_sync_t* msg = (cmd_sync_t*)p_data;
    set_timestamp(msg->timestamp);
    
    // Feedback
    size_t frame_size = protocol_calculate_frame_size(len);
    uint8_t* buffer = (uint8_t*)alloca(frame_size);
    protocol_pack_data_to_buffer(CMD_SYNC, p_data, len, buffer);
    usb_interface_send(buffer, frame_size);
}

PROTOCOL_CALLBACK_FUNCTION(reset_device)
{
    HAL_NVIC_SystemReset();
}

PROTOCOL_CALLBACK_FUNCTION(read_app_ip)
{
    struct app_manage *app = get_current_app();
    cmd_read_app_id_feedback_t msg;
    msg.timestamp = get_timestamp();
    msg.app_id = app->app_id;
    size_t frame_size = protocol_calculate_frame_size(sizeof(cmd_read_app_id_feedback_t));
    uint8_t* buffer = (uint8_t*)alloca(frame_size);
    protocol_pack_data_to_buffer(CMD_READ_APP_ID_FEEDBACK, (uint8_t*)&msg, sizeof(cmd_read_app_id_feedback_t), buffer);
    usb_interface_send(buffer, frame_size);
}

int32_t unpack_bytes(uint8_t *buf, uint32_t len)
{
    for(int i = 0; i < len; ++i)
    {
        // 解析USB字节流
        if(protocol_unpack_byte(&unpack_stream_object, buf[i]))
        {
            // 成功解析数据帧
            dispatch_frame(unpack_stream_object.cmd_id, unpack_stream_object.data, unpack_stream_object.data_len);
        }
    }
    return len;
}

void communication_task_init(void)
{
    register_cmd_callback(CMD_SYNC, sync_machine_time);
    register_cmd_callback(CMD_RESET, reset_device);
    register_cmd_callback(CMD_READ_APP_ID, read_app_ip);
    protocol_static_create_unpack_stream(&unpack_stream_object, heap_malloc(PROTOCOL_MAX_FRAME_LENGTH), PROTOCOL_MAX_FRAME_LENGTH);
    usb_vcp_rx_callback_register(unpack_bytes);
}
