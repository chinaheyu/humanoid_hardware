#ifndef LINERA_ACTUATOR_H
#define LINERA_ACTUATOR_H
#ifdef  __cplusplus
extern "C" {
#endif
#include <stdint.h>

#pragma anon_unions

#pragma pack(push)
#pragma pack(1)

typedef struct {
    uint8_t sof[2];     // 0x55 0xAA
    uint8_t length;
    uint8_t id;
    uint8_t cmd;
    uint8_t index;
} LA_BasicFrameHeader;

typedef struct {
    uint8_t sof[2];     // 0x55 0xAA
    uint8_t length;
    uint8_t id;
    uint8_t cmd;
} LA_BroadcastFrameHeader;

typedef struct {
    uint8_t sof[2];     // 0x55 0xAA
    uint8_t length;     // 0x03
    uint8_t id;
    uint8_t cmd;
    uint8_t parameter1; // 0x00
    uint8_t parameter2;
    uint8_t check_sum;
} LA_SingleFrame;

typedef struct {
    uint8_t sof[2];     // 0x55 0xAA
    uint8_t length;     // 0x11
    uint8_t id;
    uint8_t cmd;        // 0x04
    uint8_t parameter1; // 0x00
    uint8_t parameter2; // 0x22
    union {
        uint16_t target_position;
        struct {
            uint8_t target_position_low;
            uint8_t target_position_high;
        };
    };
    union {
        uint16_t current_position;
        struct {
            uint8_t current_position_low;
            uint8_t current_position_high;
        };
    };
    uint8_t temperature;
    union {
        uint16_t current;
        struct {
            uint8_t current_low;
            uint8_t current_high;
        };
    };
    uint8_t force_sensor_low;
    uint8_t error_code;
    uint8_t force_sensor_high;
    union {
        uint16_t internal_data1;
        struct {
            uint8_t internal_data1_low;
            uint8_t internal_data1_high;
        };
    };
    union {
        uint16_t internal_data2;
        struct {
            uint8_t internal_data2_low;
            uint8_t internal_data2_high;
        };
    };
    uint8_t check_sum;
} LA_InformationFeedbackFrame;

#pragma pack(pop)


typedef enum {
    LA_STEP_HEADER_LOW = 0,
    LA_STEP_HEADER_HIGH = 1,
    LA_STEP_FRAME_LENGTH = 2,
    LA_STEP_ID_CMD_INDEX_DATA = 3,
    LA_STEP_CHECK_SUM = 4
} LA_UnpackStep;


typedef struct {
    LA_UnpackStep step;
    uint8_t buffer[256];
    uint32_t pointer;
    uint8_t length;
} LA_UnpackStream;


typedef struct
{
    uint8_t id;
    uint16_t target_position;
    uint16_t current_position;
    uint8_t temperature;
    uint16_t force_sensor;
    uint8_t error_code;
    uint16_t internal_data1;
    uint16_t internal_data2;
} LA_StateFeedback;


void LA_FreeRTOS_Init(void);
void LA_SendCommand(uint8_t id, uint8_t cmd, uint8_t index, uint8_t* data, uint8_t data_size);
void LA_BroadcastCommand(uint8_t cmd, uint8_t* data, uint8_t data_size);
void LA_SendSingleCommand(uint8_t id, uint8_t parameter);
uint32_t LA_UnpackByte(LA_UnpackStream* stream, uint8_t byte);
void LA_InitializeUnpackStream(LA_UnpackStream* stream);

int LA_Read(uint8_t id, uint8_t index, uint8_t size, uint32_t timeout, uint8_t* result);
int LA_Write(uint8_t id, uint8_t index, uint8_t* data, uint8_t size, uint32_t timeout);
int LA_WriteSilent(uint8_t id, uint8_t index, uint8_t* data, uint8_t size);
int LA_SetTarget(uint8_t id, uint16_t target, uint32_t timeout);
int LA_SetTargetSilent(uint8_t id, uint16_t target);
int LA_Follow(uint8_t id, uint16_t target, uint32_t timeout);
int LA_FollowSilent(uint8_t id, uint16_t target);
int LA_BroadcastTargets(uint8_t* ids, uint16_t* targets, uint32_t num);
int LA_BroadcastFollows(uint8_t* ids, uint16_t* targets, uint32_t num);

int LA_Enable(uint8_t id, uint32_t timeout);
int LA_Stop(uint8_t id, uint32_t timeout);
int LA_Pause(uint8_t id, uint32_t timeout);
int LA_SaveParameters(uint8_t id, uint32_t timeout);
int LA_QueryState(uint8_t id, uint32_t timeout);
int LA_ClearError(uint8_t id, uint32_t timeout);

int LA_GetStateFeedback(uint8_t id, LA_StateFeedback* feedback, uint32_t timeout);

#ifdef  __cplusplus
}
#endif
#endif // LINERA_ACTUATOR_H
