#ifndef PROTOCOL_PROTOCOL_DEFINITION_H
#define PROTOCOL_PROTOCOL_DEFINITION_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include "protocol_lite.h"

#pragma pack(push)
#pragma pack(1)

//************************************
// RPC 0x00
//************************************

// ECHO
#define CMD_ECHO_REQUEST (0x0000u)
#define CMD_ECHO_RESPONSE (0x0001u)

// READ UID
#define CMD_READ_UID_REQUEST (0x0002u)
#define CMD_READ_UID_RESPONSE (0x0003u)
typedef struct 
{
    uint8_t uid[12];
} cmd_read_uid_response_t;

// READ TEMPERATURE
#define CMD_READ_TEMPERATURE_REQUEST (0x0004u)
#define CMD_READ_TEMPERATURE_RESPONSE (0x0005u)
typedef struct 
{
    float temperature;
} cmd_read_temperature_response_t;

// CMD_LINEAR_ACTUATOR_RESPONSE
#define CMD_LINEAR_ACTUATOR_RESPONSE (0x0006u)
typedef struct
{
    uint8_t id;
    uint16_t target_position;
    uint16_t current_position;
    uint8_t temperature;
    uint16_t force_sensor;
    uint8_t error_code;
    uint16_t internal_data1;
    uint16_t internal_data2;
} cmd_linear_actuator_feedback_t;

// LINEAR_ACTUATOR_SET_TARGET
#define CMD_LINEAR_ACTUATOR_SET_TARGET_REQUEST (0x0007u)
typedef struct
{
    uint8_t id;
    uint16_t target;
} cmd_linear_actuator_set_target_t;

// LINEAR_ACTUATOR_FOLLOW
#define CMD_LINEAR_ACTUATOR_FOLLOW_REQUEST (0x0008u)
typedef struct
{
    uint8_t id;
    uint16_t target;
} cmd_linear_actuator_follow_t;

// LINEAR_ACTUATOR_ENABLE
#define CMD_LINEAR_ACTUATOR_ENABLE_REQUEST (0x0009u)
typedef struct
{
    uint8_t id;
} cmd_linear_actuator_enable_t;

// LINEAR_ACTUATOR_STOP
#define CMD_LINEAR_ACTUATOR_STOP_REQUEST (0x000au)
typedef struct
{
    uint8_t id;
} cmd_linear_actuator_stop_t;

// LINEAR_ACTUATOR_PAUSE
#define CMD_LINEAR_ACTUATOR_PAUSE_REQUEST (0x000bu)
typedef struct
{
    uint8_t id;
} cmd_linear_actuator_pause_t;

// LINEAR_ACTUATOR_SAVE_PARAMETERS
#define CMD_LINEAR_ACTUATOR_SAVE_PARAMETERS_REQUEST (0x000cu)
typedef struct
{
    uint8_t id;
} cmd_linear_actuator_save_parameters_t;

// LINEAR_ACTUATOR_QUERY_STATE
#define CMD_LINEAR_ACTUATOR_QUERY_STATE_REQUEST (0x000du)
typedef struct
{
    uint8_t id;
} cmd_linear_actuator_query_state_t;

// LINEAR_ACTUATOR_CLEAR_ERROR
#define CMD_LINEAR_ACTUATOR_CLEAR_ERROR_REQUEST (0x000eu)
typedef struct
{
    uint8_t id;
} cmd_linear_actuator_clear_error_t;

//************************************
// Message 0x01
//************************************

// HEART freq=2Hz
#define CMD_HEART (0x0101u)

// WRITE_CONSOLE
#define CMD_WRITE_CONSOLE (0x0102u)

// CONSOLE_OUTPUT
#define CMD_CONSOLE_OUTPUT (0x0103u)

// SET_MAESTRO_CHANNEL
#define CMD_SET_MAESTRO_CHANNEL (0x0104u)
typedef struct 
{
    uint8_t channel;
    uint16_t target;    //units: quarter-microseconds
} cmd_set_maestro_channel_t;

// SET_MAESTRO_ALL_CHANNEL
#define CMD_SET_MAESTRO_ALL_CHANNEL (0x0105u)
typedef struct 
{
    uint16_t targets[24];    //units: quarter-microseconds
} cmd_set_maestro_all_channel_t;

// LINEAR_ACTUATOR_SET_TARGET_SILENT
#define CMD_LINEAR_ACTUATOR_SET_TARGET_SILENT (0x0106u)
typedef struct 
{
    uint8_t id;
    uint16_t target;
} cmd_linear_actuator_set_target_silent_t;

// LINEAR_ACTUATOR_FOLLOW_SILENT
#define CMD_LINEAR_ACTUATOR_FOLLOW_SILENT (0x0107u)
typedef struct 
{
    uint8_t id;
    uint16_t target;
} cmd_linear_actuator_follow_silent_t;

// LINEAR_ACTUATOR_BROADCAST_TARGETS
#define CMD_LINEAR_ACTUATOR_BROADCAST_TARGETS (0x0108u)
typedef struct 
{
    uint8_t num;
    uint8_t ids[10];
    uint16_t targets[10];
} cmd_linear_actuator_broadcast_targets_t;

// LINEAR_ACTUATOR_BROADCAST_FOLLOWS
#define CMD_LINEAR_ACTUATOR_BROADCAST_FOLLOWS (0x0109u)
typedef struct 
{
    uint8_t num;
    uint8_t ids[10];
    uint16_t targets[10];
} cmd_linear_actuator_broadcast_follows_t;

#pragma pack(pop)


#ifdef __cplusplus
}
#endif

#endif //PROTOCOL_PROTOCOL_DEFINITION_H
