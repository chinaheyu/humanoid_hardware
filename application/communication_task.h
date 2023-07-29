#ifndef __COMMUNICATION_TASK_H__
#define __COMMUNICATION_TASK_H__

#include "protocol.h"
#include "ahrs.h"

void gyro_feedback(struct ahrs_sensor* sensor);
void communication_task_init(void);

#endif // __COMMUNICATION_TASK_H__
