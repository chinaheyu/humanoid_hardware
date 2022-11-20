#include "protocol_definition.h"
#include "communicate_task.h"
#include "read_uid.h"
#include "bsp_adc.h"
#include "cmsis_os.h"
#include "maestro.h"
#include "linear_actuator.h"


void feedback_to_cmd(LA_StateFeedback* feedback, cmd_linear_actuator_feedback_t* res)
{
    res->id = feedback->id;
    res->target_position = feedback->target_position;
    res->current_position = feedback->current_position;
    res->temperature = feedback->temperature;
    res->force_sensor = feedback->force_sensor;
    res->error_code = feedback->error_code;
    res->internal_data1 = feedback->internal_data1;
    res->internal_data2 = feedback->internal_data2;
}

PROTOCOL_CALLBACK_FUNCTION(cmd_echo_callback)
{
    send_cmd_with_data(CMD_ECHO_RESPONSE, p_data, len);
}

PROTOCOL_CALLBACK_FUNCTION(cmd_read_uid_callback)
{
    cmd_read_uid_response_t res;
    
    read_unique_device_ID(res.uid, UNIQUE_ID_BYTE_SIZE);
    
    send_cmd_with_data(CMD_READ_UID_RESPONSE, (uint8_t*)&res, sizeof(cmd_read_uid_response_t));
}

PROTOCOL_CALLBACK_FUNCTION(cmd_read_temperature_callback)
{
    cmd_read_temperature_response_t res;
    
    res.temperature = get_cpu_temperature();
    
    send_cmd_with_data(CMD_READ_TEMPERATURE_RESPONSE, (uint8_t*)&res, sizeof(cmd_read_temperature_response_t));
}

PROTOCOL_CALLBACK_FUNCTION(cmd_write_console_callback)
{
    extern uint32_t shell_rx_callback(uint8_t *buff, uint16_t len);
    shell_rx_callback(p_data, len);
}

PROTOCOL_CALLBACK_FUNCTION(cmd_set_channel_callback)
{
    cmd_set_maestro_channel_t* msg = (cmd_set_maestro_channel_t*)p_data;
    MAESTRO_SetTarget(msg->channel, msg->target);
}

PROTOCOL_CALLBACK_FUNCTION(cmd_set_all_channel_callback)
{
    cmd_set_maestro_all_channel_t* msg = (cmd_set_maestro_all_channel_t*)p_data;
    MAESTRO_SetMultipleTargets(24, 0, msg->targets);
}

PROTOCOL_CALLBACK_FUNCTION(cmd_linear_actuator_set_target_callback)
{
    cmd_linear_actuator_set_target_t* req = (cmd_linear_actuator_set_target_t*)p_data;
    LA_StateFeedback feedback;
    
    if(LA_SetTarget(req->id, req->target, 100))
    {
        if(LA_GetStateFeedback(req->id, &feedback, 100))
        {
            cmd_linear_actuator_feedback_t res;
            
            feedback_to_cmd(&feedback, &res);
            
            send_cmd_with_data(CMD_LINEAR_ACTUATOR_RESPONSE, (uint8_t*)&res, sizeof(cmd_linear_actuator_feedback_t));
        }
    }
}

PROTOCOL_CALLBACK_FUNCTION(cmd_linear_actuator_follow_callback)
{
    cmd_linear_actuator_follow_t* req = (cmd_linear_actuator_follow_t*)p_data;
    LA_StateFeedback feedback;
    
    if(LA_Follow(req->id, req->target, 100))
    {
        if(LA_GetStateFeedback(req->id, &feedback, 100))
        {
            cmd_linear_actuator_feedback_t res;
            
            feedback_to_cmd(&feedback, &res);
            
            send_cmd_with_data(CMD_LINEAR_ACTUATOR_RESPONSE, (uint8_t*)&res, sizeof(cmd_linear_actuator_feedback_t));
        }
    }
}

PROTOCOL_CALLBACK_FUNCTION(cmd_linear_actuator_enable_callback)
{
    cmd_linear_actuator_enable_t* req = (cmd_linear_actuator_enable_t*)p_data;
    LA_StateFeedback feedback;
    
    if(LA_Enable(req->id, 100))
    {
        if(LA_GetStateFeedback(req->id, &feedback, 100))
        {
            cmd_linear_actuator_feedback_t res;
            
            feedback_to_cmd(&feedback, &res);
            
            send_cmd_with_data(CMD_LINEAR_ACTUATOR_RESPONSE, (uint8_t*)&res, sizeof(cmd_linear_actuator_feedback_t));
        }
    }
}

PROTOCOL_CALLBACK_FUNCTION(cmd_linear_actuator_stop_callback)
{
    cmd_linear_actuator_stop_t* req = (cmd_linear_actuator_stop_t*)p_data;
    LA_StateFeedback feedback;
    
    if(LA_Stop(req->id, 100))
    {
        if(LA_GetStateFeedback(req->id, &feedback, 100))
        {
            cmd_linear_actuator_feedback_t res;
            
            feedback_to_cmd(&feedback, &res);
            
            send_cmd_with_data(CMD_LINEAR_ACTUATOR_RESPONSE, (uint8_t*)&res, sizeof(cmd_linear_actuator_feedback_t));
        }
    }
}

PROTOCOL_CALLBACK_FUNCTION(cmd_linear_actuator_pause_callback)
{
    cmd_linear_actuator_pause_t* req = (cmd_linear_actuator_pause_t*)p_data;
    LA_StateFeedback feedback;
    
    if(LA_Pause(req->id, 100))
    {
        if(LA_GetStateFeedback(req->id, &feedback, 100))
        {
            cmd_linear_actuator_feedback_t res;
            
            feedback_to_cmd(&feedback, &res);
            
            send_cmd_with_data(CMD_LINEAR_ACTUATOR_RESPONSE, (uint8_t*)&res, sizeof(cmd_linear_actuator_feedback_t));
        }
    }
}

PROTOCOL_CALLBACK_FUNCTION(cmd_linear_actuator_save_parameters_callback)
{
    cmd_linear_actuator_save_parameters_t* req = (cmd_linear_actuator_save_parameters_t*)p_data;
    LA_StateFeedback feedback;
    
    if(LA_SaveParameters(req->id, 100))
    {
        if(LA_GetStateFeedback(req->id, &feedback, 100))
        {
            cmd_linear_actuator_feedback_t res;
            
            feedback_to_cmd(&feedback, &res);
            
            send_cmd_with_data(CMD_LINEAR_ACTUATOR_RESPONSE, (uint8_t*)&res, sizeof(cmd_linear_actuator_feedback_t));
        }
    }
}

PROTOCOL_CALLBACK_FUNCTION(cmd_linear_actuator_query_state_callback)
{
    cmd_linear_actuator_query_state_t* req = (cmd_linear_actuator_query_state_t*)p_data;
    LA_StateFeedback feedback;
    
    if(LA_QueryState(req->id, 100))
    {
        if(LA_GetStateFeedback(req->id, &feedback, 100))
        {
            cmd_linear_actuator_feedback_t res;
            
            feedback_to_cmd(&feedback, &res);
            
            send_cmd_with_data(CMD_LINEAR_ACTUATOR_RESPONSE, (uint8_t*)&res, sizeof(cmd_linear_actuator_feedback_t));
        }
    }
}

PROTOCOL_CALLBACK_FUNCTION(cmd_linear_actuator_clear_error_callback)
{
    cmd_linear_actuator_clear_error_t* req = (cmd_linear_actuator_clear_error_t*)p_data;
    LA_StateFeedback feedback;
    
    if(LA_ClearError(req->id, 100))
    {
        if(LA_GetStateFeedback(req->id, &feedback, 100))
        {
            cmd_linear_actuator_feedback_t res;
            
            feedback_to_cmd(&feedback, &res);
            
            send_cmd_with_data(CMD_LINEAR_ACTUATOR_RESPONSE, (uint8_t*)&res, sizeof(cmd_linear_actuator_feedback_t));
        }
    }
}

PROTOCOL_CALLBACK_FUNCTION(cmd_linera_actuator_set_target_silent_callback)
{
    cmd_linear_actuator_set_target_silent_t* req = (cmd_linear_actuator_set_target_silent_t*)p_data;
    
    LA_SetTargetSilent(req->id, req->target);
}

PROTOCOL_CALLBACK_FUNCTION(cmd_linera_actuator_follow_silent_callback)
{
    cmd_linear_actuator_follow_silent_t* req = (cmd_linear_actuator_follow_silent_t*)p_data;
    
    LA_FollowSilent(req->id, req->target);
}

PROTOCOL_CALLBACK_FUNCTION(cmd_linera_actuator_broadcast_targets_callback)
{
    cmd_linear_actuator_broadcast_targets_t* req = (cmd_linear_actuator_broadcast_targets_t*)p_data;
    
    LA_BroadcastTargets(req->ids, req->targets, req->num);
}

PROTOCOL_CALLBACK_FUNCTION(cmd_linera_actuator_broadcast_follows_callback)
{
    cmd_linear_actuator_broadcast_follows_t* req = (cmd_linear_actuator_broadcast_follows_t*)p_data;
    
    LA_BroadcastFollows(req->ids, req->targets, req->num);
}

void register_all_callbacks(void)
{
    register_cmd_callback(CMD_ECHO_REQUEST, cmd_echo_callback, -1, 0);
    register_cmd_callback(CMD_READ_UID_REQUEST, cmd_read_uid_callback, -1, 0);
    register_cmd_callback(CMD_READ_TEMPERATURE_REQUEST, cmd_read_temperature_callback, -1, 0);
    register_cmd_callback(CMD_WRITE_CONSOLE, cmd_write_console_callback, -1, 0);
    register_cmd_callback(CMD_SET_MAESTRO_CHANNEL, cmd_set_channel_callback, -1, 0);
    register_cmd_callback(CMD_SET_MAESTRO_ALL_CHANNEL, cmd_set_all_channel_callback, -1, 0);
    register_cmd_callback(CMD_LINEAR_ACTUATOR_SET_TARGET_REQUEST, cmd_linear_actuator_set_target_callback, -1, 0);
    register_cmd_callback(CMD_LINEAR_ACTUATOR_FOLLOW_REQUEST, cmd_linear_actuator_follow_callback, -1, 0);
    register_cmd_callback(CMD_LINEAR_ACTUATOR_ENABLE_REQUEST, cmd_linear_actuator_enable_callback, -1, 0);
    register_cmd_callback(CMD_LINEAR_ACTUATOR_STOP_REQUEST, cmd_linear_actuator_stop_callback, -1, 0);
    register_cmd_callback(CMD_LINEAR_ACTUATOR_PAUSE_REQUEST, cmd_linear_actuator_pause_callback, -1, 0);
    register_cmd_callback(CMD_LINEAR_ACTUATOR_SAVE_PARAMETERS_REQUEST, cmd_linear_actuator_save_parameters_callback, -1, 0);
    register_cmd_callback(CMD_LINEAR_ACTUATOR_QUERY_STATE_REQUEST, cmd_linear_actuator_query_state_callback, -1, 0);
    register_cmd_callback(CMD_LINEAR_ACTUATOR_CLEAR_ERROR_REQUEST, cmd_linear_actuator_clear_error_callback, -1, 0);
    register_cmd_callback(CMD_LINEAR_ACTUATOR_SET_TARGET_SILENT, cmd_linera_actuator_set_target_silent_callback, -1, 0);
    register_cmd_callback(CMD_LINEAR_ACTUATOR_FOLLOW_SILENT, cmd_linera_actuator_follow_silent_callback, -1, 0);
    register_cmd_callback(CMD_LINEAR_ACTUATOR_BROADCAST_TARGETS, cmd_linera_actuator_broadcast_targets_callback, -1, 0);
    register_cmd_callback(CMD_LINEAR_ACTUATOR_BROADCAST_FOLLOWS, cmd_linera_actuator_broadcast_follows_callback, -1, 0);


}
