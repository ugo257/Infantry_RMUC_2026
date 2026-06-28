#ifndef YAW_JBC_H
#define YAW_JBC_H

#include "stdint.h"

extern volatile uint8_t g_yaw_jbc_enable;
extern volatile uint8_t g_yaw_jbc_apply_enable;
extern volatile uint8_t g_yaw_jbc_competition_mode;
extern volatile uint8_t g_yaw_jbc_k_update_allowed;
extern volatile uint8_t g_yaw_jbc_flash_load_valid;
extern volatile uint8_t g_yaw_jbc_flash_pending_save;
extern volatile uint8_t g_yaw_jbc_flash_last_write_ok;

extern volatile uint32_t g_yaw_jbc_k_skip_count;
extern volatile uint32_t g_yaw_jbc_flash_write_count;
extern volatile uint32_t g_yaw_jbc_flash_crc_fail_count;

extern volatile float g_yaw_jbc_last_k_update_s;
extern volatile float g_yaw_jbc_q_theta;
extern volatile float g_yaw_jbc_q_omega;
extern volatile float g_yaw_jbc_r_torque;
extern volatile float g_yaw_jbc_j_hat;
extern volatile float g_yaw_jbc_b_hat;
extern volatile float g_yaw_jbc_b_use;
extern volatile float g_yaw_jbc_j_use;
extern volatile float g_yaw_jbc_k_theta_use;
extern volatile float g_yaw_jbc_k_omega_use;
extern volatile float g_yaw_jbc_allow_update;
extern volatile float g_yaw_jbc_valid_window_count;

void YawJbc_LoadFromFlash(void);
void Cal_JBCTask(void);

#endif
