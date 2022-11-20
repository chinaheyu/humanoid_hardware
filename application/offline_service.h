#ifndef OFFLINE_SERVICE_H
#define OFFLINE_SERVICE_H

#include <stdint.h>

typedef void (*offline_t)(void);

int register_offline_callback(offline_t online_func, offline_t offline_func, uint32_t offline_time);
void delete_offline_callback(int id);
void offline_hook(int id);

#endif // OFFLINE_SERVICE_H
