#ifndef UNICOMM_H
#define UNICOMM_H

#include <stdbool.h>
#include <stdint.h>

#include "robot_types.h"

#define UNICOMM_FRAME_HEAD           0xFFu
#define UNICOMM_FRAME_TYPE_CTRL_CMD  0x0Du
#define UNICOMM_FRAME_TYPE_UPLOAD    0xFDu

#define UNICOMM_CTRL_FRAME_LEN       (sizeof(Chassis_Ctrl_Cmd_s_uart) + 3u)
#define UNICOMM_UPLOAD_FRAME_LEN     (sizeof(Chassis_Upload_Data_s_uart) + 3u)

enum {
    UNICOMM_ITEM_HEAD_LEN = 2u,
    UNICOMM_ITEM_CHECKSUM_LEN = 1u,
    UNICOMM_ITEM_PAYLOAD_U8_LEN = 1u,
    UNICOMM_ITEM_PAYLOAD_32BIT_LEN = 4u,
    // 单项通信帧长度随payload类型变化: U8为4字节，float/int32为7字节。
    UNICOMM_ITEM_FRAME_MIN_LEN = 4u,
    UNICOMM_ITEM_FRAME_MAX_LEN = 7u,
    UNICOMM_ITEM_FRAME_LEN = UNICOMM_ITEM_FRAME_MAX_LEN,
};

typedef enum
{
    UNICOMM_PAYLOAD_FLOAT = 0,
    UNICOMM_PAYLOAD_I32,
    UNICOMM_PAYLOAD_U8,
} UniCommPayloadType_e;

typedef struct
{
    uint8_t head0;
    uint8_t head1;
    uint8_t payload[4];
    uint8_t payload_len;
} UniCommItemFrame_s;

uint8_t UniCommChecksum(const uint8_t *data, uint16_t len);

void UniCommFloatToBytes(float value, uint8_t *bytes);
float UniCommBytesToFloat(const uint8_t *bytes);//将4字节数组转换为float

uint16_t UniCommPackItemFrame(uint8_t head0, uint8_t head1, const void *value, UniCommPayloadType_e type, uint8_t *dst, uint16_t dst_len);
uint16_t UniCommPayloadLen(UniCommPayloadType_e type);
uint16_t UniCommItemFrameLen(UniCommPayloadType_e type);
bool UniCommUnpackItemFrame(const uint8_t *src, uint16_t src_len, UniCommPayloadType_e type, UniCommItemFrame_s *dst);

uint16_t UniCommPackChassisCtrl(const Chassis_Ctrl_Cmd_s_uart *src, uint8_t *dst, uint16_t dst_len);
bool UniCommUnpackChassisCtrl(const uint8_t *src, uint16_t src_len, Chassis_Ctrl_Cmd_s_uart *dst);

uint16_t UniCommPackChassisUpload(const Chassis_Upload_Data_s_uart *src, uint8_t *dst, uint16_t dst_len);
bool UniCommUnpackChassisUpload(const uint8_t *src, uint16_t src_len, Chassis_Upload_Data_s_uart *dst);

#endif // UNICOMM_H
