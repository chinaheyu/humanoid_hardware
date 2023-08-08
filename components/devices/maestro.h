#ifndef MAESTRO_H
#define MAESTRO_H
#ifdef  __cplusplus
extern "C" {
#endif
#include <stdint.h>

uint16_t MAESTRO_GetTarget(uint8_t channel);

// units: quarter-microseconds
void MAESTRO_SetTarget(uint8_t channel, uint16_t target);

// units: quarter-microseconds
void MAESTRO_SetMultipleTargets(uint8_t number_of_targets, uint8_t first_channel_number, uint16_t* targets);

// units: (0.25 ¦Ìs)/(10 ms)
void MAESTRO_SetSpeed(uint8_t channel, uint16_t speed);

// units: (0.25 ¦Ìs)/(10 ms)/(80 ms)
void MAESTRO_SetAcceleration(uint8_t channel, uint16_t acceleration);

// units: 1/48 ¦Ìs
void MAESTRO_SetPWM(uint16_t on_time, uint16_t period);

#ifdef  __cplusplus
}  
#endif
#endif // MAESTRO_H
