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

#include "gyro.h"
#include "errno.h"

#define LOG_TAG "drv.gyro"
#define LOG_OUTPUT_LEVEL  LOG_INFO
#include "log.h"

/**
  * @brief  gyro device initialize
  * @param
  * @retval error code
  */
int32_t gyro_device_init(struct gyro_device *gyro, char *name)
{
    device_assert(gyro != NULL);

    ((device_t)gyro)->type = DEVICE_GYRO;

    device_init(&(gyro->parent), name);
    return E_OK;
}


