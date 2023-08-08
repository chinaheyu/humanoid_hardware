#ifndef __COMMUNICATION_TASK_H__
#define __COMMUNICATION_TASK_H__

#include "protocol.h"
#include "ahrs.h"
#include <alloca.h>
#include "usbd_cdc_if.h"

#define register_cmd_callback(cmd_id, callback) __register_cmd_callback((cmd_id), (callback), (#callback))
#define PROTOCOL_CALLBACK_FUNCTION(name) void name(uint16_t cmd_id, uint8_t *p_data, uint16_t len)

typedef void (*protocol_cmd_callback_t)(uint16_t cmd_id, uint8_t *p_data, uint16_t len);
void __register_cmd_callback(uint16_t cmd_id, protocol_cmd_callback_t callback, const char *callback_name);

void communication_task_init(void);

#endif // __COMMUNICATION_TASK_H__
