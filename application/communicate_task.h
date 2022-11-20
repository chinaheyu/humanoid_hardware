/*
Api:
register_cmd_callback
send_cmd_with_data
remote_procedure_call
*/

#ifndef COMMUNICATE_TASK_H
#define COMMUNICATE_TASK_H

#ifdef  __cplusplus
extern "C" {
#endif


#include <stdint.h>
#include "protocol_definition.h"


typedef void (*protocol_cmd_callback_t)(uint16_t cmd_id, uint8_t *p_data, uint16_t len, void* parameter);
void __register_cmd_callback(uint16_t cmd_id, protocol_cmd_callback_t callback, short life, void* parameter, const char *callback_name);

// RPC返回数据
typedef struct _RPCResponse
{
    uint8_t data[PROTOCOL_DATA_MAX_SIZE];
    uint16_t len;
} RPCResponse;

// 注册回调函数
#define register_cmd_callback(cmd_id, callback, life, parameter) __register_cmd_callback((cmd_id), (callback), (life), (parameter), (#callback))

// 删除回调函数
void remove_cmd_callback(uint16_t cmd_id);

// 发送命令和数据
int send_cmd_with_data(uint16_t cmd_id, uint8_t *p_data, uint16_t len);

// 进行远程调用 禁止在通讯线程中执行
int remote_procedure_call(  uint16_t request_cmd_id, uint8_t *p_request_data, uint16_t request_len,
                            uint16_t response_cmd_id, RPCResponse *p_rpc_response, uint32_t timeout);

// 快捷定义回调函数
#define PROTOCOL_CALLBACK_FUNCTION(name) void name(uint16_t cmd_id, uint8_t *p_data, uint16_t len, void* parameter)


#ifdef  __cplusplus
}  
#endif

#endif // COMMUNICATE_TASK_H
