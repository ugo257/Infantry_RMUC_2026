#include "unicomm.h"

#include <string.h>

#define CTRL_FLOAT_BASE_INDEX        2u
#define CTRL_FLAGS_BASE_INDEX        (sizeof(Chassis_Ctrl_Cmd_s_uart) - 7u)
#define CTRL_CHECKSUM_INDEX          (sizeof(Chassis_Ctrl_Cmd_s_uart) + 2u)

#define UPLOAD_FLOAT_BASE_INDEX      2u
#define UPLOAD_COLOR_INDEX           (UPLOAD_FLOAT_BASE_INDEX + 32u)
#define UPLOAD_HOLE_YAW_READY_INDEX  (UPLOAD_COLOR_INDEX + 1u)
#define UPLOAD_CHECKSUM_INDEX        (sizeof(Chassis_Upload_Data_s_uart) + 2u)

void UniCommFloatToBytes(float value, uint8_t *bytes)
{
    uint32_t raw = 0;
    memcpy(&raw, &value, sizeof(float));
    bytes[0] = (uint8_t)(raw & 0xFFu);
    bytes[1] = (uint8_t)((raw >> 8) & 0xFFu);
    bytes[2] = (uint8_t)((raw >> 16) & 0xFFu);
    bytes[3] = (uint8_t)((raw >> 24) & 0xFFu);
}

float UniCommBytesToFloat(const uint8_t *bytes)
{
    uint32_t raw = 0;
    raw |= ((uint32_t)bytes[0]);
    raw |= ((uint32_t)bytes[1] << 8);
    raw |= ((uint32_t)bytes[2] << 16);
    raw |= ((uint32_t)bytes[3] << 24);

    float value = 0.0f;
    memcpy(&value, &raw, sizeof(float));
    return value;
}

static void int32_to_bytes_be(int32_t value, uint8_t *bytes)
{
    bytes[0] = (uint8_t)((value >> 24) & 0xFFu);
    bytes[1] = (uint8_t)((value >> 16) & 0xFFu);
    bytes[2] = (uint8_t)((value >> 8) & 0xFFu);
    bytes[3] = (uint8_t)(value & 0xFFu);
}

static int32_t bytes_to_int32_be(const uint8_t *bytes)
{
    return ((int32_t)bytes[0] << 24) |
           ((int32_t)bytes[1] << 16) |
           ((int32_t)bytes[2] << 8) |
           ((int32_t)bytes[3]);
}

uint8_t UniCommChecksum(const uint8_t *data, uint16_t len)
{
    uint8_t checksum = 0u;
    for (uint16_t i = 0; i < len; ++i) {
        checksum = (uint8_t)(checksum + data[i]);
    }
    return checksum;
}

static void int32_to_bytes_le(int32_t value, uint8_t *bytes)
{
    bytes[0] = (uint8_t)(value & 0xFFu);
    bytes[1] = (uint8_t)((value >> 8) & 0xFFu);
    bytes[2] = (uint8_t)((value >> 16) & 0xFFu);
    bytes[3] = (uint8_t)((value >> 24) & 0xFFu);
}

uint16_t UniCommPayloadLen(UniCommPayloadType_e type)
{
    switch (type) {
        case UNICOMM_PAYLOAD_FLOAT:
        case UNICOMM_PAYLOAD_I32:
            return UNICOMM_ITEM_PAYLOAD_32BIT_LEN;
        case UNICOMM_PAYLOAD_U8:
            return UNICOMM_ITEM_PAYLOAD_U8_LEN;
        default:
            return 0u;
    }
}

uint16_t UniCommItemFrameLen(UniCommPayloadType_e type)
{
    uint16_t payload_len = UniCommPayloadLen(type);

    if (payload_len == 0u) {
        return 0u;
    }

    return (uint16_t)(UNICOMM_ITEM_HEAD_LEN + payload_len + UNICOMM_ITEM_CHECKSUM_LEN);
}

uint16_t UniCommPackItemFrame(uint8_t head0, uint8_t head1, const void *value, UniCommPayloadType_e type, uint8_t *dst, uint16_t dst_len)
{
    uint16_t payload_len = UniCommPayloadLen(type);
    uint16_t frame_len = UniCommItemFrameLen(type);

    if (value == NULL || dst == NULL || frame_len == 0u || dst_len < frame_len) {
        return 0u;
    }

    memset(dst, 0, frame_len);
    dst[0] = head0;
    dst[1] = head1;

    switch (type) {
        case UNICOMM_PAYLOAD_FLOAT:
            UniCommFloatToBytes(*(const float *)value, dst + 2u);
            break;
        case UNICOMM_PAYLOAD_I32: {
            int32_t i32_value = 0;
            memcpy(&i32_value, value, sizeof(i32_value));
            int32_to_bytes_le(i32_value, dst + 2u);
            break;
        }
        case UNICOMM_PAYLOAD_U8:
            dst[2] = *(const uint8_t *)value;
            break;
        default:
            return 0u;
    }

    dst[UNICOMM_ITEM_HEAD_LEN + payload_len] = UniCommChecksum(dst, (uint16_t)(frame_len - 1u));
    return frame_len;
}

bool UniCommUnpackItemFrame(const uint8_t *src, uint16_t src_len, UniCommPayloadType_e type, UniCommItemFrame_s *dst)
{
    uint16_t payload_len = UniCommPayloadLen(type);
    uint16_t frame_len = UniCommItemFrameLen(type);

    if (src == NULL || dst == NULL || frame_len == 0u || src_len < frame_len) {
        return false;
    }
    if (UniCommChecksum(src, (uint16_t)(frame_len - 1u)) != src[frame_len - 1u]) {
        return false;
    }

    dst->head0 = src[0];
    dst->head1 = src[1];
    memset(dst->payload, 0, sizeof(dst->payload));
    memcpy(dst->payload, src + UNICOMM_ITEM_HEAD_LEN, payload_len);
    dst->payload_len = (uint8_t)payload_len;
    return true;
}

uint16_t UniCommPackChassisCtrl(const Chassis_Ctrl_Cmd_s_uart *src, uint8_t *dst, uint16_t dst_len)
{
    if (src == NULL || dst == NULL || dst_len < UNICOMM_CTRL_FRAME_LEN) {
        return 0u;
    }

    memset(dst, 0, UNICOMM_CTRL_FRAME_LEN);
    dst[0] = UNICOMM_FRAME_HEAD;
    dst[1] = UNICOMM_FRAME_TYPE_CTRL_CMD;

    UniCommFloatToBytes(src->vx, dst + CTRL_FLOAT_BASE_INDEX + 0u);
    UniCommFloatToBytes(src->vy, dst + CTRL_FLOAT_BASE_INDEX + 4u);
    UniCommFloatToBytes(src->wz, dst + CTRL_FLOAT_BASE_INDEX + 8u);
    UniCommFloatToBytes(src->yaw_control, dst + CTRL_FLOAT_BASE_INDEX + 12u);
    UniCommFloatToBytes(src->yaw_angle, dst + CTRL_FLOAT_BASE_INDEX + 16u);
    UniCommFloatToBytes(src->yaw_gyro, dst + CTRL_FLOAT_BASE_INDEX + 20u);
    UniCommFloatToBytes(src->offset_angle, dst + CTRL_FLOAT_BASE_INDEX + 24u);
    UniCommFloatToBytes(src->gimbal_error_angle, dst + CTRL_FLOAT_BASE_INDEX + 28u);
    int32_to_bytes_be(src->shoot_count, dst + CTRL_FLOAT_BASE_INDEX + 32u);
    UniCommFloatToBytes(src->nuc_yaw, dst + CTRL_FLOAT_BASE_INDEX + 36u);
    UniCommFloatToBytes(src->yaw_vel, dst + CTRL_FLOAT_BASE_INDEX + 40u);  // 视觉yaw速度前馈

    dst[CTRL_FLAGS_BASE_INDEX + 0u] = (uint8_t)src->nuc_mode;
    dst[CTRL_FLAGS_BASE_INDEX + 1u] = src->UI_SendFlag;
    dst[CTRL_FLAGS_BASE_INDEX + 2u] = src->superCap_flag;
    dst[CTRL_FLAGS_BASE_INDEX + 3u] = src->reset_flag;
    dst[CTRL_FLAGS_BASE_INDEX + 4u] = (uint8_t)src->friction_mode;
    dst[CTRL_FLAGS_BASE_INDEX + 5u] = (uint8_t)src->shoot_mode;
    dst[CTRL_FLAGS_BASE_INDEX + 6u] = (uint8_t)src->load_mode;
    dst[CTRL_FLAGS_BASE_INDEX + 7u] = (uint8_t)src->chassis_mode;
    dst[CTRL_FLAGS_BASE_INDEX + 8u] = (uint8_t)src->gimbal_mode;

    dst[CTRL_CHECKSUM_INDEX] = UniCommChecksum(dst, CTRL_CHECKSUM_INDEX);
    return UNICOMM_CTRL_FRAME_LEN;
}

bool UniCommUnpackChassisCtrl(const uint8_t *src, uint16_t src_len, Chassis_Ctrl_Cmd_s_uart *dst)
{
    if (src == NULL || dst == NULL || src_len < UNICOMM_CTRL_FRAME_LEN) {
        return false;
    }
    if (src[0] != UNICOMM_FRAME_HEAD || src[1] != UNICOMM_FRAME_TYPE_CTRL_CMD) {
        return false;
    }
    if (UniCommChecksum(src, CTRL_CHECKSUM_INDEX) != src[CTRL_CHECKSUM_INDEX]) {
        return false;
    }

    dst->vx = UniCommBytesToFloat(src + CTRL_FLOAT_BASE_INDEX + 0u);
    dst->vy = UniCommBytesToFloat(src + CTRL_FLOAT_BASE_INDEX + 4u);
    dst->wz = UniCommBytesToFloat(src + CTRL_FLOAT_BASE_INDEX + 8u);
    dst->yaw_control = UniCommBytesToFloat(src + CTRL_FLOAT_BASE_INDEX + 12u);
    dst->yaw_angle = UniCommBytesToFloat(src + CTRL_FLOAT_BASE_INDEX + 16u);
    dst->yaw_gyro = UniCommBytesToFloat(src + CTRL_FLOAT_BASE_INDEX + 20u);
    dst->offset_angle = UniCommBytesToFloat(src + CTRL_FLOAT_BASE_INDEX + 24u);
    dst->gimbal_error_angle = UniCommBytesToFloat(src + CTRL_FLOAT_BASE_INDEX + 28u);
    dst->shoot_count = bytes_to_int32_be(src + CTRL_FLOAT_BASE_INDEX + 32u);
    dst->nuc_yaw = UniCommBytesToFloat(src + CTRL_FLOAT_BASE_INDEX + 36u);
    dst->yaw_vel = UniCommBytesToFloat(src + CTRL_FLOAT_BASE_INDEX + 40u);  // 视觉yaw速度前馈

    dst->nuc_mode = (nuc_mode_e)src[CTRL_FLAGS_BASE_INDEX + 0u];
    dst->UI_SendFlag = src[CTRL_FLAGS_BASE_INDEX + 1u];
    dst->superCap_flag = src[CTRL_FLAGS_BASE_INDEX + 2u];
    dst->reset_flag = src[CTRL_FLAGS_BASE_INDEX + 3u];
    dst->friction_mode = (friction_mode_e)src[CTRL_FLAGS_BASE_INDEX + 4u];
    dst->shoot_mode = (shoot_mode_e)src[CTRL_FLAGS_BASE_INDEX + 5u];
    dst->load_mode = (loader_mode_e)src[CTRL_FLAGS_BASE_INDEX + 6u];
    dst->chassis_mode = (chassis_mode_e)src[CTRL_FLAGS_BASE_INDEX + 7u];
    dst->gimbal_mode = (gimbal_mode_e)src[CTRL_FLAGS_BASE_INDEX + 8u];
    return true;
}

uint16_t UniCommPackChassisUpload(const Chassis_Upload_Data_s_uart *src, uint8_t *dst, uint16_t dst_len)
{
    if (src == NULL || dst == NULL || dst_len < UNICOMM_UPLOAD_FRAME_LEN) {
        return 0u;
    }

    memset(dst, 0, UNICOMM_UPLOAD_FRAME_LEN);
    dst[0] = UNICOMM_FRAME_HEAD;
    dst[1] = UNICOMM_FRAME_TYPE_UPLOAD;

    UniCommFloatToBytes(src->yaw_motor_single_round_angle, dst + UPLOAD_FLOAT_BASE_INDEX + 0u);
    UniCommFloatToBytes(src->yaw_total_angle, dst + UPLOAD_FLOAT_BASE_INDEX + 4u);
    UniCommFloatToBytes(src->yaw_ecd, dst + UPLOAD_FLOAT_BASE_INDEX + 8u);
    UniCommFloatToBytes(src->chassis_pitch_angle, dst + UPLOAD_FLOAT_BASE_INDEX + 12u);
    UniCommFloatToBytes(src->chassis_yaw_gyro, dst + UPLOAD_FLOAT_BASE_INDEX + 16u);
    UniCommFloatToBytes(src->initial_speed, dst + UPLOAD_FLOAT_BASE_INDEX + 20u);
    UniCommFloatToBytes(src->yaw_angle_pidout, dst + UPLOAD_FLOAT_BASE_INDEX + 24u);
    UniCommFloatToBytes(src->yaw_motor_vel, dst + UPLOAD_FLOAT_BASE_INDEX + 28u);

    dst[UPLOAD_COLOR_INDEX] = (uint8_t)src->color;
    dst[UPLOAD_HOLE_YAW_READY_INDEX] = src->hole_yaw_ready;
    dst[UPLOAD_CHECKSUM_INDEX] = UniCommChecksum(dst, UPLOAD_CHECKSUM_INDEX);
    return UNICOMM_UPLOAD_FRAME_LEN;
}

bool UniCommUnpackChassisUpload(const uint8_t *src, uint16_t src_len, Chassis_Upload_Data_s_uart *dst)
{
    if (src == NULL || dst == NULL || src_len < UNICOMM_UPLOAD_FRAME_LEN) {
        return false;
    }
    if (src[0] != UNICOMM_FRAME_HEAD || src[1] != UNICOMM_FRAME_TYPE_UPLOAD) {
        return false;
    }
    if (UniCommChecksum(src, UPLOAD_CHECKSUM_INDEX) != src[UPLOAD_CHECKSUM_INDEX]) {
        return false;
    }

    dst->yaw_motor_single_round_angle = UniCommBytesToFloat(src + UPLOAD_FLOAT_BASE_INDEX + 0u);
    dst->yaw_total_angle = UniCommBytesToFloat(src + UPLOAD_FLOAT_BASE_INDEX + 4u);
    dst->yaw_ecd = UniCommBytesToFloat(src + UPLOAD_FLOAT_BASE_INDEX + 8u);
    dst->chassis_pitch_angle = UniCommBytesToFloat(src + UPLOAD_FLOAT_BASE_INDEX + 12u);
    dst->chassis_yaw_gyro = UniCommBytesToFloat(src + UPLOAD_FLOAT_BASE_INDEX + 16u);
    dst->initial_speed = UniCommBytesToFloat(src + UPLOAD_FLOAT_BASE_INDEX + 20u);
    dst->yaw_angle_pidout = UniCommBytesToFloat(src + UPLOAD_FLOAT_BASE_INDEX + 24u);
    dst->yaw_motor_vel = UniCommBytesToFloat(src + UPLOAD_FLOAT_BASE_INDEX + 28u);
    dst->color = (version_color)src[UPLOAD_COLOR_INDEX];
    dst->hole_yaw_ready = src[UPLOAD_HOLE_YAW_READY_INDEX];
    return true;
}
