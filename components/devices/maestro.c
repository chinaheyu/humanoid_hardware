#include "maestro.h"
#include "drv_uart.h"
#include "cmsis_os.h"

// Use Compact protocol
// https://www.pololu.com/docs/0J40/5.e

#define MAESTRO_CHAMMEL_NUM 24
#define MAESTRO_MESSAGE_BUF_LENTH (3 + 2 * MAESTRO_CHAMMEL_NUM)
static uint8_t message_buf[MAESTRO_MESSAGE_BUF_LENTH];
uint16_t servo_value[MAESTRO_CHAMMEL_NUM];

uint16_t MAESTRO_GetTarget(uint8_t channel)
{
    return servo_value[channel];
}

// units: quarter-microseconds
void MAESTRO_SetTarget(uint8_t channel, uint16_t target)
{
    servo_value[channel] = target;
    
    message_buf[0] = 0x84; // Command byte: Set Target.
    message_buf[1] = channel; // First data byte holds channel number.
    message_buf[2] = target & 0x7F; // Second byte holds the lower 7 bits of target.
    message_buf[3] = (target >> 7) & 0x7F;   // Third data byte holds the bits 7-13 of target.
    
    usart6_transmit(message_buf, 4);
}

// units: quarter-microseconds
void MAESTRO_SetMultipleTargets(uint8_t number_of_targets, uint8_t first_channel_number, uint16_t* targets)
{
    message_buf[0] = 0x9F;
    message_buf[1] = number_of_targets;
    message_buf[2] = first_channel_number;
    
    for(int i = 0; i < number_of_targets; ++i)
    {
        servo_value[i] = targets[i];
        message_buf[3 + 2 * i] = targets[i] & 0x7F;
        message_buf[4 + 2 * i] = (targets[i] >> 7) & 0x7F;
    }
    
    usart6_transmit(message_buf, 3 + number_of_targets * 2);
}

// units: (0.25 ¦Ìs)/(10 ms), RDS3235 = 51.28
void MAESTRO_SetSpeed(uint8_t channel, uint16_t speed)
{
    message_buf[0] = 0x87;
    message_buf[1] = channel;
    message_buf[2] = speed & 0x7F;
    message_buf[3] = (speed >> 7) & 0x7F;
    
    usart6_transmit(message_buf, 4);
}

// units: (0.25 ¦Ìs)/(10 ms)/(80 ms)
void MAESTRO_SetAcceleration(uint8_t channel, uint16_t acceleration)
{
    message_buf[0] = 0x89;
    message_buf[1] = channel;
    message_buf[2] = acceleration & 0x7F;
    message_buf[3] = (acceleration >> 7) & 0x7F;
    
    usart6_transmit(message_buf, 4);
}

// units: 1/48 ¦Ìs
void MAESTRO_SetPWM(uint16_t on_time, uint16_t period)
{
    message_buf[0] = 0x8A;
    message_buf[1] = on_time & 0x7F;
    message_buf[2] = (on_time >> 7) & 0x7F;
    message_buf[3] = period & 0x7F;
    message_buf[4] = (period >> 7) & 0x7F;
    
    usart6_transmit(message_buf, 5);
}

