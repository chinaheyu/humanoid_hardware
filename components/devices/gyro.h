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

#ifndef __GYRO_H__
#define __GYRO_H__

#ifdef SINGLE_GYRO_H_GLOBAL
    #define SINGLE_GYRO_H_EXTERN
#else
    #define SINGLE_GYRO_H_EXTERN extern
#endif

#include "stdint.h"
#include "device.h"

struct gyro_device
{
    struct device parent;
    float roll;
    float pitch;
    float yaw;
};

int32_t gyro_device_init(struct gyro_device *gyro, char *name);


#endif // __GYRO_H__
