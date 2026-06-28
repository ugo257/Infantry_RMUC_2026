#include "yaw_jbc.h"

#include "bsp_flash.h"
#include "crc16.h"
#include "lqr_eso.h"
#include "robot_types.h"

#include "math.h"
#include "string.h"

#define YAW_JBC_TASK_DT_DEFAULT_S        0.001f
#define YAW_JBC_WINDOW_S                 0.050f
#define YAW_JBC_JB_UPDATE_S              5.0f
#define YAW_JBC_K_UPDATE_S               20.0f
#define YAW_JBC_OMEGA_LPF_TAU_S          0.010f
#define YAW_JBC_B_USE_TAU_S              3.0f
#define YAW_JBC_J_USE_TAU_S              8.0f
#define YAW_JBC_SIGN_FREEZE_S            0.100f
#define YAW_JBC_MIN_OMEGA_RAD_S          0.35f
#define YAW_JBC_MIN_PHI                  0.005f
#define YAW_JBC_B_MU                     0.06f
#define YAW_JBC_J_MU                     0.04f
#define YAW_JBC_B_EPS                    1.0e-5f
#define YAW_JBC_LS_DET_EPS               1.0e-7f
#define YAW_JBC_B_STEP_LIMIT             0.03f
#define YAW_JBC_J_STEP_LIMIT             0.003f
#define YAW_JBC_B_MIN                    0.05f
#define YAW_JBC_B_MAX                    0.90f
#define YAW_JBC_J_MIN                    0.035f
#define YAW_JBC_J_MAX                    0.090f
#define YAW_JBC_K_THETA_MIN              1.0f
#define YAW_JBC_K_THETA_MAX              150.0f
#define YAW_JBC_K_OMEGA_MIN              0.0f
#define YAW_JBC_K_OMEGA_MAX              10.0f
#define YAW_JBC_Q_THETA_DEFAULT          400.0f
#define YAW_JBC_Q_OMEGA_DEFAULT          0.5f
#define YAW_JBC_R_TORQUE_DEFAULT         1.0f
#define YAW_JBC_FLASH_MAGIC              0x594A424BU /* "YJBK" */
#define YAW_JBC_FLASH_VERSION            1U
#define YAW_JBC_FLASH_START_ADDR         (ADDR_FLASH_SECTOR_11 + 0x1000U)
#define YAW_JBC_FLASH_END_ADDR           ADDR_FLASH_SECTOR_12

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t word_len;
    uint32_t sequence;
    float j_use;
    float b_use;
    float c_use;
    float k_theta;
    float k_omega;
    float q_theta;
    float q_omega;
    float r_torque;
    uint16_t crc16;
    uint16_t reserved;
} YawJbcFlashRecord_t;

typedef struct {
    uint32_t last_seq;
    uint8_t initialized;
    uint8_t q_initialized;
    uint8_t window_valid;
    float time_s;
    float omega_f;
    float omega_start;
    float tau_int;
    float omega_int;
    float abs_omega_int;
    float win_t;
    float jb_timer_s;
    float k_timer_s;
    float sign_freeze_s;
    int8_t omega_sign_last;
    float sum_phi2;
    float sum_phi_y;
    float sum_domega2;
    float sum_domega_phi;
    float sum_domega_y;
    uint32_t valid_window_count;
    uint32_t flash_next_addr;
    uint32_t flash_sequence;
    uint8_t flash_loaded_once;
    uint8_t flash_pending;
    YawJbcFlashRecord_t flash_pending_record;
} YawJbcState_t;

volatile uint8_t g_yaw_jbc_enable = 0U;
volatile uint8_t g_yaw_jbc_apply_enable = 0U;
volatile uint8_t g_yaw_jbc_competition_mode = 0U;
volatile uint8_t g_yaw_jbc_k_update_allowed = 0U;
volatile uint8_t g_yaw_jbc_flash_load_valid = 0U;
volatile uint8_t g_yaw_jbc_flash_pending_save = 0U;
volatile uint8_t g_yaw_jbc_flash_last_write_ok = 0U;

volatile uint32_t g_yaw_jbc_k_skip_count = 0U;
volatile uint32_t g_yaw_jbc_flash_write_count = 0U;
volatile uint32_t g_yaw_jbc_flash_crc_fail_count = 0U;

volatile float g_yaw_jbc_last_k_update_s = 0.0f;
volatile float g_yaw_jbc_q_theta = YAW_JBC_Q_THETA_DEFAULT;
volatile float g_yaw_jbc_q_omega = YAW_JBC_Q_OMEGA_DEFAULT;
volatile float g_yaw_jbc_r_torque = YAW_JBC_R_TORQUE_DEFAULT;
volatile float g_yaw_jbc_j_hat = 0.0f;
volatile float g_yaw_jbc_b_hat = 0.0f;
volatile float g_yaw_jbc_b_use = 0.0f;
volatile float g_yaw_jbc_j_use = 0.0f;
volatile float g_yaw_jbc_k_theta_use = 0.0f;
volatile float g_yaw_jbc_k_omega_use = 0.0f;
volatile float g_yaw_jbc_allow_update = 0.0f;
volatile float g_yaw_jbc_valid_window_count = 0.0f;

static YawJbcState_t yaw_jbc;

static float YawJbcClamp(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static uint8_t YawJbcFloatValid(float value)
{
    return (isfinite(value) != 0) ? 1U : 0U;
}

static uint8_t YawJbcRecordRangeValid(const YawJbcFlashRecord_t *record)
{
    if (record == NULL) {
        return 0U;
    }
    if (YawJbcFloatValid(record->j_use) == 0U ||
        YawJbcFloatValid(record->b_use) == 0U ||
        YawJbcFloatValid(record->c_use) == 0U ||
        YawJbcFloatValid(record->k_theta) == 0U ||
        YawJbcFloatValid(record->k_omega) == 0U ||
        YawJbcFloatValid(record->q_theta) == 0U ||
        YawJbcFloatValid(record->q_omega) == 0U ||
        YawJbcFloatValid(record->r_torque) == 0U) {
        return 0U;
    }
    if (record->j_use < YAW_JBC_J_MIN ||
        record->j_use > YAW_JBC_J_MAX ||
        record->b_use < YAW_JBC_B_MIN ||
        record->b_use > YAW_JBC_B_MAX ||
        record->k_theta <= 0.0f ||
        record->k_omega < 0.0f ||
        record->q_theta <= 0.0f ||
        record->q_omega < 0.0f ||
        record->r_torque <= 1.0e-6f) {
        return 0U;
    }
    return 1U;
}

static uint16_t YawJbcRecordCrc(const YawJbcFlashRecord_t *record)
{
    return crc_16((const uint8_t *)record,
                  (uint16_t)(sizeof(YawJbcFlashRecord_t) - 4U));
}

static uint8_t YawJbcRecordValid(const YawJbcFlashRecord_t *record)
{
    if (record == NULL ||
        record->magic != YAW_JBC_FLASH_MAGIC ||
        record->version != YAW_JBC_FLASH_VERSION ||
        record->word_len != (uint16_t)(sizeof(YawJbcFlashRecord_t) / sizeof(uint32_t)) ||
        YawJbcRecordRangeValid(record) == 0U) {
        return 0U;
    }
    return (record->crc16 == YawJbcRecordCrc(record)) ? 1U : 0U;
}

static void YawJbcApplyPlant(float j_use, float b_use)
{
    g_yaw_j = j_use;
    g_yaw_b = b_use;
    g_yaw_auto_j = j_use;
    g_yaw_auto_b = b_use;
}

static void YawJbcInitQFromCurrentK(void)
{
    const float r = (g_yaw_jbc_r_torque > 1.0e-6f) ? g_yaw_jbc_r_torque : 1.0f;
    const float j = (g_yaw_jbc_j_use > YAW_JBC_J_MIN) ? g_yaw_jbc_j_use : g_yaw_j;
    const float b = (g_yaw_jbc_b_use > YAW_JBC_B_MIN) ? g_yaw_jbc_b_use : g_yaw_b;
    const float k_theta = (g_lqr_k_theta > 0.0f) ? g_lqr_k_theta : 1.0f;
    const float k_omega = (g_lqr_k_omega > 0.0f) ? g_lqr_k_omega : 0.0f;
    float q_omega;

    if (yaw_jbc.q_initialized != 0U) {
        return;
    }

    g_yaw_jbc_r_torque = r;
    if (g_yaw_jbc_q_theta <= 0.0f || YawJbcFloatValid(g_yaw_jbc_q_theta) == 0U) {
        g_yaw_jbc_q_theta = k_theta * k_theta * r;
    }
    if (g_yaw_jbc_q_omega < 0.0f || YawJbcFloatValid(g_yaw_jbc_q_omega) == 0U) {
        g_yaw_jbc_q_omega = 0.0f;
    }
    if (g_yaw_jbc_q_omega <= 0.0f) {
        q_omega = ((k_omega + b) * (k_omega + b) - b * b - 2.0f * j * k_theta) * r;
        g_yaw_jbc_q_omega = (q_omega > 0.0f) ? q_omega : 0.0f;
    }
    yaw_jbc.q_initialized = 1U;
}

static void YawJbcResetWindow(void)
{
    yaw_jbc.omega_start = yaw_jbc.omega_f;
    yaw_jbc.tau_int = 0.0f;
    yaw_jbc.omega_int = 0.0f;
    yaw_jbc.abs_omega_int = 0.0f;
    yaw_jbc.win_t = 0.0f;
    yaw_jbc.window_valid = 0U;
}

static void YawJbcPublishCoreState(void)
{
    g_yaw_jbc_valid_window_count = (float)yaw_jbc.valid_window_count;
    g_yaw_jbc_flash_pending_save = yaw_jbc.flash_pending;
}

static void YawJbcPrepareInitialState(void)
{
    float j = g_yaw_j;
    float b = g_yaw_b;

    if (j < YAW_JBC_J_MIN || j > YAW_JBC_J_MAX || YawJbcFloatValid(j) == 0U) {
        j = g_yaw_auto_j;
    }
    if (b < YAW_JBC_B_MIN || b > YAW_JBC_B_MAX || YawJbcFloatValid(b) == 0U) {
        b = g_yaw_auto_b;
    }
    j = YawJbcClamp(j, YAW_JBC_J_MIN, YAW_JBC_J_MAX);
    b = YawJbcClamp(b, YAW_JBC_B_MIN, YAW_JBC_B_MAX);

    g_yaw_jbc_j_use = j;
    g_yaw_jbc_b_use = b;
    g_yaw_jbc_j_hat = j;
    g_yaw_jbc_b_hat = b;
    g_yaw_jbc_k_theta_use = g_lqr_k_theta;
    g_yaw_jbc_k_omega_use = g_lqr_k_omega;
    YawJbcInitQFromCurrentK();
}

static uint32_t YawJbcFindNextFlashAddress(uint32_t best_addr, uint8_t has_best)
{
    const uint32_t step = (uint32_t)sizeof(YawJbcFlashRecord_t);
    uint32_t addr;

    if (has_best != 0U) {
        addr = best_addr + step;
        if (addr + step <= YAW_JBC_FLASH_END_ADDR &&
            (*(const uint32_t *)addr) == 0xFFFFFFFFU) {
            return addr;
        }
    }

    for (addr = YAW_JBC_FLASH_START_ADDR;
         addr + step <= YAW_JBC_FLASH_END_ADDR;
         addr += step) {
        if ((*(const uint32_t *)addr) == 0xFFFFFFFFU) {
            return addr;
        }
    }
    return YAW_JBC_FLASH_START_ADDR;
}

void YawJbc_LoadFromFlash(void)
{
    YawJbcFlashRecord_t record;
    YawJbcFlashRecord_t best_record;
    uint32_t best_addr = YAW_JBC_FLASH_START_ADDR;
    uint8_t has_best = 0U;
    uint32_t addr;

    if (yaw_jbc.flash_loaded_once != 0U) {
        return;
    }
    yaw_jbc.flash_loaded_once = 1U;
    if (g_yaw_jbc_enable == 0U && g_yaw_jbc_apply_enable == 0U) {
        g_yaw_jbc_flash_load_valid = 0U;
        return;
    }
    memset(&best_record, 0, sizeof(best_record));

    for (addr = YAW_JBC_FLASH_START_ADDR;
         addr + sizeof(YawJbcFlashRecord_t) <= YAW_JBC_FLASH_END_ADDR;
         addr += (uint32_t)sizeof(YawJbcFlashRecord_t)) {
        memcpy(&record, (const void *)addr, sizeof(record));
        if (record.magic == 0xFFFFFFFFU) {
            continue;
        }
        if (YawJbcRecordValid(&record) != 0U) {
            if (has_best == 0U || record.sequence > best_record.sequence) {
                best_record = record;
                best_addr = addr;
                has_best = 1U;
            }
        } else {
            g_yaw_jbc_flash_crc_fail_count++;
        }
    }

    if (has_best != 0U) {
        g_yaw_jbc_j_use = best_record.j_use;
        g_yaw_jbc_b_use = best_record.b_use;
        g_yaw_jbc_j_hat = best_record.j_use;
        g_yaw_jbc_b_hat = best_record.b_use;
        g_yaw_jbc_k_theta_use = best_record.k_theta;
        g_yaw_jbc_k_omega_use = best_record.k_omega;
        g_yaw_jbc_q_theta = best_record.q_theta;
        g_yaw_jbc_q_omega = best_record.q_omega;
        g_yaw_jbc_r_torque = best_record.r_torque;
        g_yaw_tau_coulomb_nm = best_record.c_use;
        g_yaw_auto_tau_coulomb_nm = best_record.c_use;
        YawJbcApplyPlant(best_record.j_use, best_record.b_use);
        g_lqr_k_theta = best_record.k_theta;
        g_lqr_k_omega = best_record.k_omega;
        g_lqr_auto_k_theta = best_record.k_theta;
        g_lqr_auto_k_omega = best_record.k_omega;
        yaw_jbc.flash_sequence = best_record.sequence;
        g_yaw_jbc_flash_load_valid = 1U;
        yaw_jbc.q_initialized = 1U;
    } else {
        g_yaw_jbc_flash_load_valid = 0U;
    }
    yaw_jbc.flash_next_addr = YawJbcFindNextFlashAddress(best_addr, has_best);
}

static void YawJbcRequestFlashSave(void)
{
    YawJbcFlashRecord_t record;

    memset(&record, 0, sizeof(record));
    record.magic = YAW_JBC_FLASH_MAGIC;
    record.version = YAW_JBC_FLASH_VERSION;
    record.word_len = (uint16_t)(sizeof(YawJbcFlashRecord_t) / sizeof(uint32_t));
    record.sequence = yaw_jbc.flash_sequence + 1U;
    record.j_use = g_yaw_jbc_j_use;
    record.b_use = g_yaw_jbc_b_use;
    record.c_use = g_yaw_tau_coulomb_nm;
    record.k_theta = g_yaw_jbc_k_theta_use;
    record.k_omega = g_yaw_jbc_k_omega_use;
    record.q_theta = g_yaw_jbc_q_theta;
    record.q_omega = g_yaw_jbc_q_omega;
    record.r_torque = g_yaw_jbc_r_torque;
    record.reserved = 0U;
    record.crc16 = YawJbcRecordCrc(&record);

    if (YawJbcRecordRangeValid(&record) == 0U) {
        return;
    }
    yaw_jbc.flash_pending_record = record;
    yaw_jbc.flash_pending = 1U;
    g_yaw_jbc_flash_pending_save = 1U;
}

static void YawJbcFlashService(void)
{
    const uint32_t step = (uint32_t)sizeof(YawJbcFlashRecord_t);

    if (yaw_jbc.flash_pending == 0U) {
        g_yaw_jbc_flash_pending_save = 0U;
        return;
    }

    if (yaw_jbc.flash_next_addr + step > YAW_JBC_FLASH_END_ADDR ||
        (*(const uint32_t *)yaw_jbc.flash_next_addr) != 0xFFFFFFFFU) {
        flash_erase_address(ADDR_FLASH_SECTOR_11, 1U);
        yaw_jbc.flash_next_addr = YAW_JBC_FLASH_START_ADDR;
    }

    if (flash_write_single_address(
            yaw_jbc.flash_next_addr,
            (uint32_t *)&yaw_jbc.flash_pending_record,
            (uint32_t)(sizeof(YawJbcFlashRecord_t) / sizeof(uint32_t))) == 0) {
        yaw_jbc.flash_sequence = yaw_jbc.flash_pending_record.sequence;
        yaw_jbc.flash_next_addr += step;
        yaw_jbc.flash_pending = 0U;
        g_yaw_jbc_flash_last_write_ok = 1U;
        g_yaw_jbc_flash_write_count++;
    } else {
        g_yaw_jbc_flash_last_write_ok = 0U;
    }
    g_yaw_jbc_flash_pending_save = yaw_jbc.flash_pending;
}

static uint8_t YawJbcCalcK(void)
{
    const float j = YawJbcClamp(g_yaw_jbc_j_use, YAW_JBC_J_MIN, YAW_JBC_J_MAX);
    const float b = YawJbcClamp(g_yaw_jbc_b_use, YAW_JBC_B_MIN, YAW_JBC_B_MAX);
    const float r = (g_yaw_jbc_r_torque > 1.0e-6f) ? g_yaw_jbc_r_torque : 1.0f;
    const float q_theta = (g_yaw_jbc_q_theta > 0.0f) ? g_yaw_jbc_q_theta : 1.0f;
    const float q_omega = (g_yaw_jbc_q_omega > 0.0f) ? g_yaw_jbc_q_omega : 0.0f;
    const float k_theta_raw = sqrtf(q_theta / r);
    const float omega_term = b * b + 2.0f * j * k_theta_raw + q_omega / r;
    float k_theta;
    float k_omega;

    if (omega_term < 0.0f ||
        YawJbcFloatValid(k_theta_raw) == 0U ||
        YawJbcFloatValid(omega_term) == 0U) {
        return 0U;
    }

    k_theta = YawJbcClamp(k_theta_raw, YAW_JBC_K_THETA_MIN, YAW_JBC_K_THETA_MAX);
    k_omega = YawJbcClamp(-b + sqrtf(omega_term),
                          YAW_JBC_K_OMEGA_MIN,
                          YAW_JBC_K_OMEGA_MAX);
    if (YawJbcFloatValid(k_theta) == 0U || YawJbcFloatValid(k_omega) == 0U) {
        return 0U;
    }

    g_yaw_jbc_k_theta_use = k_theta;
    g_yaw_jbc_k_omega_use = k_omega;
    g_lqr_k_theta = k_theta;
    g_lqr_k_omega = k_omega;
    g_lqr_auto_k_theta = k_theta;
    g_lqr_auto_k_omega = k_omega;
    g_yaw_jbc_last_k_update_s = yaw_jbc.time_s;
    return 1U;
}

static uint8_t YawJbcAllowUpdate(const YawAutoAimDebugRecord_s *d)
{
    uint8_t allow = 1U;

    if (d == NULL) {
        return 0U;
    }
    if (d->feedback_ok == 0U) {
        allow = 0U;
    }
    if (d->auto_aim_mode != 0U || d->nuc_mode == version_control) {
        allow = 0U;
    }
    if (d->gimbal_mode != GIMBAL_GYRO_MODE &&
        d->gimbal_mode != GIMBAL_MOTOR_MODE) {
        allow = 0U;
    }
    if (d->soft_limit_active != 0U ||
        d->dm_limit_active != 0U ||
        d->slew_limit_active != 0U) {
        allow = 0U;
    }
    if (fabsf(d->omega_meas_rad_s) < YAW_JBC_MIN_OMEGA_RAD_S) {
        allow = 0U;
    }
    if (yaw_jbc.sign_freeze_s > 0.0f) {
        allow = 0U;
    }
    return allow;
}

static void YawJbcUpdateSignFreeze(float omega, float dt)
{
    int8_t sign = 0;

    if (omega > YAW_JBC_MIN_OMEGA_RAD_S) {
        sign = 1;
    } else if (omega < -YAW_JBC_MIN_OMEGA_RAD_S) {
        sign = -1;
    }

    if (sign != 0 && yaw_jbc.omega_sign_last != 0 && sign != yaw_jbc.omega_sign_last) {
        yaw_jbc.sign_freeze_s = YAW_JBC_SIGN_FREEZE_S;
    }
    if (sign != 0) {
        yaw_jbc.omega_sign_last = sign;
    }
    if (yaw_jbc.sign_freeze_s > 0.0f) {
        yaw_jbc.sign_freeze_s -= dt;
        if (yaw_jbc.sign_freeze_s < 0.0f) {
            yaw_jbc.sign_freeze_s = 0.0f;
        }
    }
}

static void YawJbcUpdateWindow(float dt,
                               float omega_meas,
                               float tau_cmd_applied,
                               uint8_t allow_update)
{
    const float lpf_alpha = dt / (YAW_JBC_OMEGA_LPF_TAU_S + dt);
    float tau_c;
    float tau_nf;

    yaw_jbc.omega_f += lpf_alpha * (omega_meas - yaw_jbc.omega_f);
    YawJbcUpdateSignFreeze(yaw_jbc.omega_f, dt);

    if (allow_update == 0U) {
        YawJbcResetWindow();
        return;
    }
    if (yaw_jbc.window_valid == 0U) {
        YawJbcResetWindow();
        yaw_jbc.window_valid = 1U;
    }

    {
        const float omega_s =
            (g_yaw_coulomb_smooth_omega > 1.0e-3f) ? g_yaw_coulomb_smooth_omega : 1.0e-3f;
        tau_c = g_yaw_tau_coulomb_nm * tanhf(yaw_jbc.omega_f / omega_s);
    }
    tau_nf = tau_cmd_applied - tau_c;

    yaw_jbc.tau_int += tau_nf * dt;
    yaw_jbc.omega_int += yaw_jbc.omega_f * dt;
    yaw_jbc.abs_omega_int += fabsf(yaw_jbc.omega_f) * dt;
    yaw_jbc.win_t += dt;

    if (yaw_jbc.win_t >= YAW_JBC_WINDOW_S) {
        const float d_omega = yaw_jbc.omega_f - yaw_jbc.omega_start;
        const float phi = yaw_jbc.omega_int;
        const float y = yaw_jbc.tau_int;
        const float omega_abs_avg =
            (yaw_jbc.win_t > 1.0e-6f) ? (yaw_jbc.abs_omega_int / yaw_jbc.win_t) : 0.0f;

        if (omega_abs_avg > YAW_JBC_MIN_OMEGA_RAD_S &&
            fabsf(phi) > YAW_JBC_MIN_PHI) {
            yaw_jbc.sum_domega2 += d_omega * d_omega;
            yaw_jbc.sum_domega_phi += d_omega * phi;
            yaw_jbc.sum_domega_y += d_omega * y;
            yaw_jbc.sum_phi2 += phi * phi;
            yaw_jbc.sum_phi_y += phi * y;
            yaw_jbc.valid_window_count++;
        }
        YawJbcResetWindow();
    }
}

static void YawJbcUpdateJbAndMaybeK(float dt)
{
    const float b_use_alpha = dt / YAW_JBC_B_USE_TAU_S;
    const float j_use_alpha = dt / YAW_JBC_J_USE_TAU_S;

    yaw_jbc.jb_timer_s += dt;
    yaw_jbc.k_timer_s += dt;
    g_yaw_jbc_j_use += j_use_alpha * (g_yaw_jbc_j_hat - g_yaw_jbc_j_use);
    g_yaw_jbc_j_use = YawJbcClamp(g_yaw_jbc_j_use, YAW_JBC_J_MIN, YAW_JBC_J_MAX);
    g_yaw_jbc_b_use += b_use_alpha * (g_yaw_jbc_b_hat - g_yaw_jbc_b_use);
    g_yaw_jbc_b_use = YawJbcClamp(g_yaw_jbc_b_use, YAW_JBC_B_MIN, YAW_JBC_B_MAX);

    if (yaw_jbc.jb_timer_s >= YAW_JBC_JB_UPDATE_S) {
        if (yaw_jbc.sum_phi2 > YAW_JBC_B_EPS && yaw_jbc.valid_window_count > 0U) {
            const float det =
                yaw_jbc.sum_domega2 * yaw_jbc.sum_phi2 -
                yaw_jbc.sum_domega_phi * yaw_jbc.sum_domega_phi;
            float b_est = (yaw_jbc.sum_phi_y -
                           g_yaw_jbc_j_hat * yaw_jbc.sum_domega_phi) /
                          yaw_jbc.sum_phi2;
            float j_est = g_yaw_jbc_j_hat;

            if (fabsf(det) > YAW_JBC_LS_DET_EPS) {
                j_est = (yaw_jbc.sum_domega_y * yaw_jbc.sum_phi2 -
                         yaw_jbc.sum_phi_y * yaw_jbc.sum_domega_phi) / det;
                b_est = (yaw_jbc.sum_domega2 * yaw_jbc.sum_phi_y -
                         yaw_jbc.sum_domega_phi * yaw_jbc.sum_domega_y) / det;
            }

            if (YawJbcFloatValid(j_est) != 0U &&
                j_est >= YAW_JBC_J_MIN &&
                j_est <= YAW_JBC_J_MAX) {
                float dJ = YAW_JBC_J_MU * (j_est - g_yaw_jbc_j_hat);
                dJ = YawJbcClamp(dJ, -YAW_JBC_J_STEP_LIMIT, YAW_JBC_J_STEP_LIMIT);
                g_yaw_jbc_j_hat = YawJbcClamp(g_yaw_jbc_j_hat + dJ,
                                              YAW_JBC_J_MIN,
                                              YAW_JBC_J_MAX);
            }

            float dB = YAW_JBC_B_MU * (b_est - g_yaw_jbc_b_hat);
            dB = YawJbcClamp(dB, -YAW_JBC_B_STEP_LIMIT, YAW_JBC_B_STEP_LIMIT);
            if (YawJbcFloatValid(b_est) != 0U &&
                b_est >= YAW_JBC_B_MIN &&
                b_est <= YAW_JBC_B_MAX) {
                g_yaw_jbc_b_hat = YawJbcClamp(g_yaw_jbc_b_hat + dB,
                                              YAW_JBC_B_MIN,
                                              YAW_JBC_B_MAX);
            }
        }
        if (g_yaw_jbc_apply_enable != 0U) {
            YawJbcApplyPlant(g_yaw_jbc_j_use, g_yaw_jbc_b_use);
        }
        yaw_jbc.sum_domega2 = 0.0f;
        yaw_jbc.sum_domega_phi = 0.0f;
        yaw_jbc.sum_domega_y = 0.0f;
        yaw_jbc.sum_phi2 = 0.0f;
        yaw_jbc.sum_phi_y = 0.0f;
        yaw_jbc.valid_window_count = 0U;
        yaw_jbc.jb_timer_s = 0.0f;
    }

    if (yaw_jbc.k_timer_s >= YAW_JBC_K_UPDATE_S) {
        g_yaw_jbc_k_update_allowed = (g_yaw_jbc_competition_mode != 0U) ? 1U : 0U;
        if (g_yaw_jbc_competition_mode != 0U) {
            if (YawJbcCalcK() != 0U) {
                YawJbcRequestFlashSave();
            }
        } else {
            g_yaw_jbc_k_skip_count++;
        }
        yaw_jbc.k_timer_s = 0.0f;
    }
}

static void YawJbcEnsureInit(void)
{
    if (yaw_jbc.initialized != 0U) {
        return;
    }
    YawJbc_LoadFromFlash();
    YawJbcPrepareInitialState();
    yaw_jbc.initialized = 1U;
}

void Cal_JBCTask(void)
{
    YawAutoAimDebugRecord_s d;
    uint32_t seq;
    float dt;
    uint8_t allow_update;

    if (g_yaw_jbc_enable == 0U) {
        return;
    }

    YawJbcEnsureInit();
    YawJbcFlashService();

    seq = g_yaw_lqreso_control_seq;
    if (seq == yaw_jbc.last_seq) {
        return;
    }
    yaw_jbc.last_seq = seq;

    memcpy(&d, (const void *)&g_yaw_auto_aim_debug_record, sizeof(d));
    dt = d.dt_s;
    if (dt < 0.0001f || dt > 0.02f || YawJbcFloatValid(dt) == 0U) {
        dt = YAW_JBC_TASK_DT_DEFAULT_S;
    }
    yaw_jbc.time_s += dt;

    allow_update = YawJbcAllowUpdate(&d);
    g_yaw_jbc_allow_update = (float)allow_update;
    YawJbcUpdateWindow(dt, d.omega_meas_rad_s, d.tau_cmd_nm, allow_update);
    YawJbcUpdateJbAndMaybeK(dt);
    YawJbcPublishCoreState();
}
