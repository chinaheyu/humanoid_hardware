#include "linear_actuator.h"
#include "bsp_usart.h"
#include "cmsis_os.h"
#include "string.h"


static uint8_t* LA_tx_buffer;
static uint8_t* LA_rx_buffer;
osEventFlagsId_t LA_EventHandle;
const osEventFlagsAttr_t LA_Event_attributes = {
  .name = "LA_Event"
};
osMutexId_t LA_RxMutexHandle;
const osMutexAttr_t LA_RxMutex_attributes = {
  .name = "LA_RxMutex"
};

uint8_t get_check_sum(uint8_t* data, uint32_t len)
{
    uint32_t sum;
    for(int i = 0; i < len; ++i)
    {
        sum += data[i];
    }
    return sum & 0xffu;
}

void append_check_sum(uint8_t* data, uint32_t len)
{
    data[len] = get_check_sum(data, len);
}

void LA_FreeRTOS_Init(void)
{
    LA_EventHandle = osEventFlagsNew(&LA_Event_attributes);
    LA_RxMutexHandle = osMutexNew(&LA_RxMutex_attributes);
    LA_tx_buffer = (uint8_t*)pvPortMalloc(256);
    LA_rx_buffer = (uint8_t*)pvPortMalloc(256);
}

uint16_t LA_GetIdAndCmd(uint8_t* frame)
{
    uint16_t id_and_cmd = frame[3];
    id_and_cmd |= (frame[4] << 8);
    return id_and_cmd;
}

uint8_t LA_GetLength(uint8_t* frame)
{
    return frame[2];
}

void LA_SendCommand(uint8_t id, uint8_t cmd, uint8_t index, uint8_t* data, uint8_t data_size)
{
    LA_BasicFrameHeader* p_header = (LA_BasicFrameHeader*)LA_tx_buffer;
    p_header->sof[0] = 0x55u;
    p_header->sof[1] = 0xaau;
    p_header->length = data_size + 2;
    p_header->id = id;
    p_header->cmd = cmd;
    p_header->index = index;
    
    memcpy(LA_tx_buffer + 6, data, data_size);
    append_check_sum(LA_tx_buffer + 2, data_size + 4);
    
    usart2_transmit(LA_tx_buffer, data_size + 7);
}

void LA_BroadcastCommand(uint8_t cmd, uint8_t* data, uint8_t data_size)
{
    LA_BroadcastFrameHeader* p_header = (LA_BroadcastFrameHeader*)LA_tx_buffer;
    p_header->sof[0] = 0x55u;
    p_header->sof[1] = 0xaau;
    p_header->length = data_size + 1;
    p_header->id = 0xffu;;
    p_header->cmd = cmd;
    
    memcpy(LA_tx_buffer + 5, data, data_size);
    append_check_sum(LA_tx_buffer + 2, data_size + 3);
    
    usart2_transmit(LA_tx_buffer, data_size + 6);
}

void LA_SendSingleCommand(uint8_t id, uint8_t parameter)
{
    LA_SingleFrame* p_header = (LA_SingleFrame*)LA_tx_buffer;
    p_header->sof[0] = 0x55u;
    p_header->sof[1] = 0xaau;
    p_header->length = 0x03u;
    p_header->id = id;
    p_header->cmd = 0x04u;
    p_header->parameter1 = 0x00u;
    p_header->parameter2 = parameter;
    
    append_check_sum(LA_tx_buffer + 2, 5);
    
    usart2_transmit(LA_tx_buffer, 8);
}

void LA_InitializeUnpackStream(LA_UnpackStream* stream)
{
    stream->step = LA_STEP_HEADER_LOW;
    stream->pointer = 0;
}

uint32_t LA_UnpackByte(LA_UnpackStream* stream, uint8_t byte)
{
    switch (stream->step)
    {
        case LA_STEP_HEADER_LOW:
        {
            if (byte == 0xaau)
            {
                stream->step = LA_STEP_HEADER_HIGH;
                stream->buffer[stream->pointer++] = byte;
            }
            else
            {
                stream->pointer = 0;
            }

            return 0;
        }

        case LA_STEP_HEADER_HIGH:
        {
            if (byte == 0xaau)
            {
                stream->step = LA_STEP_HEADER_HIGH;
                stream->buffer[stream->pointer++] = byte;
            }
            else
            {
                stream->step = LA_STEP_HEADER_LOW;
                stream->pointer = 0;
            }

            return 0;
        }

        case LA_STEP_FRAME_LENGTH:
        {
            stream->length = byte;
            stream->buffer[stream->pointer++] = byte;
            
            if(stream->length <= 251)
            {
                stream->step = LA_STEP_ID_CMD_INDEX_DATA;
            }
            else
            {
                stream->step = LA_STEP_HEADER_LOW;
                stream->pointer = 0;
            }

            return 0;
        }

        case LA_STEP_ID_CMD_INDEX_DATA:
        {
            stream->buffer[stream->pointer++] = byte;
            
            if(stream->pointer >= stream->length + 4)
            {
                stream->step = LA_STEP_CHECK_SUM;
            }

            return 0;
        }


        case LA_STEP_CHECK_SUM:
        {
            stream->buffer[stream->pointer++] = byte;
            
            stream->step = LA_STEP_HEADER_LOW;
            stream->pointer = 0;
            
            if(get_check_sum(stream->buffer + 2, stream->length + 2) == byte)
            {
                // reveived frame
                
                // one time copy
                osStatus_t result = osMutexAcquire(LA_RxMutexHandle, 10);
                if(result == osOK)
                {
                    // send event
                    osEventFlagsSet(LA_EventHandle, LA_GetIdAndCmd(LA_rx_buffer));
                    memcpy(LA_rx_buffer, stream->buffer, stream->length + 5);
                    osMutexRelease(LA_RxMutexHandle);
                }
                
                return 1;
            }
            
            return 0;
        }

        default:
        {
            stream->step = LA_STEP_HEADER_LOW;
            stream->pointer = 0;
            return 0;
        }
    }
}

int LA_WaitIdCmdFeedback(uint8_t id, uint8_t cmd, uint32_t timeout)
{
    osEventFlagsClear(LA_EventHandle, 0xFFFFFFFFU);
    
    uint32_t id_and_cmd = id | (cmd << 8);
    
    uint32_t flags = osEventFlagsWait(LA_EventHandle, id_and_cmd, osFlagsWaitAll, timeout);
    
    if(flags != id_and_cmd)
    {
        return 0;
    }
    return 1;
}

int LA_Read(uint8_t id, uint8_t index, uint8_t size, uint32_t timeout, uint8_t* result)
{
    LA_SendCommand(id, 0x01, index, &size, 1);
    
    int ret = LA_WaitIdCmdFeedback(id, 0x01, timeout);
    
    if(ret)
    {
        if(osMutexAcquire(LA_RxMutexHandle, timeout))
        {
            if(LA_GetIdAndCmd(LA_rx_buffer) == (id | (0x01 << 8)))
            {
                memcpy(result, LA_rx_buffer + 6, size);
                osMutexRelease(LA_RxMutexHandle);
                return 1;
            }
            
            osMutexRelease(LA_RxMutexHandle);
        }
    }
    
    return 0;
}

int LA_Write(uint8_t id, uint8_t index, uint8_t* data, uint8_t size, uint32_t timeout)
{
    LA_SendCommand(id, 0x02, index, data, size);
    
    return LA_WaitIdCmdFeedback(id, 0x04, timeout);
}

int LA_WriteSilent(uint8_t id, uint8_t index, uint8_t* data, uint8_t size)
{
    LA_SendCommand(id, 0x03, index, data, size);
    
    return 1;
}

int LA_SetTarget(uint8_t id, uint16_t target, uint32_t timeout)
{
    LA_SendCommand(id, 0x21, 0x37, (uint8_t*)&target, 2);
    
    return LA_WaitIdCmdFeedback(id, 0x04, timeout);
}

int LA_SetTargetSilent(uint8_t id, uint16_t target)
{
    LA_SendCommand(id, 0x03, 0x37, (uint8_t*)&target, 2);
    
    return 1;
}

int LA_Follow(uint8_t id, uint16_t target, uint32_t timeout)
{
    LA_SendCommand(id, 0x20, 0x37, (uint8_t*)&target, 2);
    
    return LA_WaitIdCmdFeedback(id, 0x04, timeout);
}

int LA_FollowSilent(uint8_t id, uint16_t target)
{
    LA_SendCommand(id, 0x19, 0x37, (uint8_t*)&target, 2);
    
    return 1;
}

int LA_BroadcastTargets(uint8_t* ids, uint16_t* targets, uint32_t num)
{
    LA_BroadcastFrameHeader* p_header = (LA_BroadcastFrameHeader*)LA_tx_buffer;
    p_header->sof[0] = 0x55u;
    p_header->sof[1] = 0xaau;
    p_header->length = 3 * num + 1;
    p_header->id = 0xffu;;
    p_header->cmd = 0xf2u;
    
    for(int i = 0; i < num; ++i)
    {
        LA_tx_buffer[5 + 3 * i] = ids[i];
        LA_tx_buffer[6 + 3 * i] = (uint8_t)(targets[i] & 0x00ff);
        LA_tx_buffer[7 + 3 * i] = (uint8_t)(targets[i] >> 8);
    }
    
    append_check_sum(LA_tx_buffer + 2, 3 * num + 3);
    
    usart2_transmit(LA_tx_buffer, 3 * num + 6);
    
    return 1;
}

int LA_BroadcastFollows(uint8_t* ids, uint16_t* targets, uint32_t num)
{
    LA_BroadcastFrameHeader* p_header = (LA_BroadcastFrameHeader*)LA_tx_buffer;
    p_header->sof[0] = 0x55u;
    p_header->sof[1] = 0xaau;
    p_header->length = 3 * num + 1;
    p_header->id = 0xffu;;
    p_header->cmd = 0xf3u;
    
    for(int i = 0; i < num; ++i)
    {
        LA_tx_buffer[5 + 3 * i] = ids[i];
        LA_tx_buffer[6 + 3 * i] = (uint8_t)(targets[i] & 0x00ff);
        LA_tx_buffer[7 + 3 * i] = (uint8_t)(targets[i] >> 8);
    }
    
    append_check_sum(LA_tx_buffer + 2, 3 * num + 3);
    
    usart2_transmit(LA_tx_buffer, 3 * num + 6);
    
    return 1;
}

int LA_Enable(uint8_t id, uint32_t timeout)
{
    LA_SendSingleCommand(id, 0x04);
    
    return LA_WaitIdCmdFeedback(id, 0x04, timeout);
}

int LA_Stop(uint8_t id, uint32_t timeout)
{
    LA_SendSingleCommand(id, 0x23);
    
    return LA_WaitIdCmdFeedback(id, 0x04, timeout);
}

int LA_Pause(uint8_t id, uint32_t timeout)
{
    LA_SendSingleCommand(id, 0x14);
    
    return LA_WaitIdCmdFeedback(id, 0x04, timeout);
}

int LA_SaveParameters(uint8_t id, uint32_t timeout)
{
    LA_SendSingleCommand(id, 0x20);
    
    return LA_WaitIdCmdFeedback(id, 0x04, timeout);
}

int LA_QueryState(uint8_t id, uint32_t timeout)
{
    LA_SendSingleCommand(id, 0x22);
    
    return LA_WaitIdCmdFeedback(id, 0x04, timeout);
}

int LA_ClearError(uint8_t id, uint32_t timeout)
{
    LA_SendSingleCommand(id, 0x1E);
    
    return LA_WaitIdCmdFeedback(id, 0x04, timeout);
}

int LA_GetStateFeedback(uint8_t id, LA_StateFeedback* feedback, uint32_t timeout)
{
    if(osMutexAcquire(LA_RxMutexHandle, timeout))
    {
        if(LA_rx_buffer[3] == id)
        {
            LA_InformationFeedbackFrame* ptr = (LA_InformationFeedbackFrame*)LA_rx_buffer;
            
            feedback->id = ptr->id;
            feedback->target_position = ptr->target_position;
            feedback->current_position = ptr->current_position;
            feedback->temperature = ptr->temperature;
            feedback->force_sensor = ptr->force_sensor_low;
            feedback->force_sensor |= ptr->force_sensor_high;
            feedback->error_code = ptr->error_code;
            feedback->internal_data1 = ptr->internal_data1;
            feedback->internal_data2 = ptr->internal_data2;
            
            osMutexRelease(LA_RxMutexHandle);
            return 1;
        }
        
        osMutexRelease(LA_RxMutexHandle);
    }
    return 0;
}
