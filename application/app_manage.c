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

#include "app_manage.h"
#include "test.h"
#include "head_control_task.h"
#include "leg_control_task.h"

#define LOG_TAG "app_manage"
#define LOG_OUTPUT_LEVEL 4
#include "log.h"

struct app_manage current_app;

struct app_table
{
    const char* app_name;
    void(*app_init_fn)(void);
}

static app_table[] = {
    {"TEST", test_task_init},
    {"HEAD_CONTROL", head_control_task_init},
    {"LEFT_LEG_CONTROL", leg_control_task_init},
    {"RIGHT_LEG_CONTROL", leg_control_task_init},
    {"WAIST_CONTROL", leg_control_task_init},
};

struct app_manage *get_current_app(void)
{
    return &current_app;
}

void app_task_init(uint8_t app_id)
{
    current_app.app_id = app_id;
    if (app_id < sizeof(app_table) / sizeof(app_table[0]))
    {
        log_i("Application init: %s", app_table[app_id].app_name);
        app_table[app_id].app_init_fn();
    }
    else
    {
        log_e("Application id not exist: %d", app_id);
    }
}

void print_app_table()
{
    for (int i = 0; i < sizeof(app_table) / sizeof(app_table[0]); i++)
    {
        log_printf("\r\n%d: %s", i, app_table[i].app_name);
    }
    log_printf("\r\nCurrent app is %s.\r\n", app_table[current_app.app_id].app_name);
}
