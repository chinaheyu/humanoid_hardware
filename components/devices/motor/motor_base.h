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

#ifndef __MOTOR_BASE_H__
#define __MOTOR_BASE_H__

#ifdef MOTOR_H_GLOBAL
    #define MOTOR_H_EXTERN
#else
    #define MOTOR_H_EXTERN extern
#endif

#include "device.h"

enum motor_type
{
    MOTOR_DM = 0,
    MOTOR_TMOTOR,
    MOTOR_ZEROERR,
    MOTOR_UNKNOW
};

struct motor_data
{
    float position;                 // rad
    float velocity;                 // rad/s
    float torque;                   // Nm
};

struct motor_device
{
    struct device parent;       // inherit device
    uint8_t id;                 // motor global id
    enum motor_type type;       // child type
    struct motor_data data;     // motor data
};

typedef struct motor_device *motor_device_t;

motor_device_t motor_find(const char *name);
int32_t motor_register(motor_device_t motor_dev, const char *name);

int float_to_uint(float x, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);

#endif // __MOTOR_BASE_H__
