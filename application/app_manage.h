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

#ifndef __APP_MANAGE__
#define __APP_MANAGE__

#include "sys.h"
#include "device.h"

struct app_manage
{
    uint8_t app_id;
    void (*dbus_rx_complete)(void);
    void (*user_key_callback)(void);
};

struct app_manage *get_current_app(void);
void app_task_init(uint8_t app_id);
void print_app_table();

#endif
