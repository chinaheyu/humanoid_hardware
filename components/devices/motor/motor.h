/****************************************************************************
 *  Copyright (C) 2020 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "motor_dm.h"
#include "motor_tmotor.h"
#include "motor_zeroerr.h"

void motor_mit_control(uint8_t id, float position, float velocity, float kp, float kd, float torque);
void motor_position_control(uint8_t id, float position);
int32_t motor_feedback(void *argc);

#endif // __MOTOR_H__
