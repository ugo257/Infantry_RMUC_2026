#include "omni_velocity_ctrl.h"

#include <math.h>
#include <string.h>

#include "general_def.h"
#include "robot_params.h"

#define OMNI_MIN_DT 0.0002f
#define OMNI_MAX_DT 0.0100f
#define OMNI_WHEEL_FORCE_LIMIT 30.0f
#define OMNI_ROLLING_DIR_Q 0.70710678f
#define OMNI_MIN_CMD_TO_SI 1.0e-9f
#define OMNI_CURRENT_OFFSET_FORCE_DEADBAND_N 1.0f
#define OMNI_MULTI_SLIP_AVG_START 0.25f
#define OMNI_MULTI_SLIP_AVG_FULL 0.75f
#define OMNI_MULTI_SLIP_MIN_WRENCH_SCALE 1.0f//0.70f
#define OMNI_MULTI_SLIP_MAX_TRIGGER 0.90f
#define OMNI_MULTI_SLIP_AVG_TRIGGER 0.45f
#define OMNI_SLIP_RECOVERY_ALPHA 0.20f
#define OMNI_SLIP_RECOVERY_CMD_EPS 1.0f
#define OMNI_SLIP_RECOVERY_FORCE_EPS_N 2.0f
#define OMNI_SLIP_RECOVERY_CURRENT_EPS_CMD 800.0f
#define OMNI_POWER_SCALE_BISECT_STEPS 4U
#define OMNI_POWER_SCALE_EPS 1.0e-5f
#define OMNI_POWER_FEEDBACK_DEFAULT_DEADBAND_W 8.0f
#define OMNI_POWER_FEEDBACK_DEFAULT_SCALE_MIN 0.55f
#define OMNI_POWER_FEEDBACK_DEFAULT_DOWN_ALPHA 0.015f
#define OMNI_POWER_FEEDBACK_DEFAULT_UP_ALPHA 0.006f
#define OMNI_POWER_FEEDBACK_DEFAULT_LPF_TAU_S 0.050f
#define OMNI_POWER_FEEDBACK_DEFAULT_LOCAL_MIN_RATIO 0.25f
#define OMNI_POWER_BUDGET_MARGIN_DEFAULT 0.92f
#define OMNI_POWER_WHEEL_SPEED_LIMIT_DEFAULT 3000.0f
#define OMNI_ROTATE_SEARCH_SCALE_MAX 100000.0f
#define OMNI_ROTATE_VXY_ACTIVE_EPS 1.0f
#define OMNI_REACHABLE_BUDGET_REL_TOL 0.05f
#define OMNI_REACHABLE_BUDGET_ABS_TOL_W 1.0f
#define OMNI_REACHABLE_DIR_DOT_MIN 0.9659258f
#define OMNI_IDEAL_CMD_TO_MPS 6.8e-5f
#define OMNI_IDEAL_CMD_TO_RADPS 1.3e-3f
#define OMNI_AW_WRENCH_ERR_TH_VXY_N 0.5f
#define OMNI_AW_WRENCH_ERR_TH_WZ_NM 0.05f

static float OmniAbs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float OmniClamp(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float OmniClampAbs(float value, float max_abs)
{
    const float limit = OmniAbs(max_abs);
    return OmniClamp(value, -limit, limit);
}

static float OmniSignNonZero(float value)
{
    return (value >= 0.0f) ? 1.0f : -1.0f;
}

static float OmniSanitizeDt(const OmniVelocityCtrl_s *ctrl, float dt)
{
    if (dt < OMNI_MIN_DT || dt > OMNI_MAX_DT) {
        return ctrl->config.control_dt_s;
    }
    return dt;
}

static float OmniWheelRadiusM(void)
{
    return RADIUS_WHEEL * 0.001f;
}

static float OmniYawLeverM(const OmniVelocityCtrl_s *ctrl)
{
    return (ctrl->config.wheel_base_mm + ctrl->config.track_width_mm) * 0.0005f * OMNI_ROLLING_DIR_Q;
}

static void OmniBuildPhysicalForceMatrix(OmniVelocityCtrl_s *ctrl)
{
    const float q = OMNI_ROLLING_DIR_Q;
    const float lever = OmniYawLeverM(ctrl);

    ctrl->force_h[OMNI_WHEEL_LF][OMNI_AXIS_VX] = q;
    ctrl->force_h[OMNI_WHEEL_LF][OMNI_AXIS_VY] = -q;
    ctrl->force_h[OMNI_WHEEL_LF][OMNI_AXIS_WZ] = -lever;

    ctrl->force_h[OMNI_WHEEL_RF][OMNI_AXIS_VX] = q;
    ctrl->force_h[OMNI_WHEEL_RF][OMNI_AXIS_VY] = q;
    ctrl->force_h[OMNI_WHEEL_RF][OMNI_AXIS_WZ] = lever;

    ctrl->force_h[OMNI_WHEEL_RB][OMNI_AXIS_VX] = q;
    ctrl->force_h[OMNI_WHEEL_RB][OMNI_AXIS_VY] = -q;
    ctrl->force_h[OMNI_WHEEL_RB][OMNI_AXIS_WZ] = lever;

    ctrl->force_h[OMNI_WHEEL_LB][OMNI_AXIS_VX] = q;
    ctrl->force_h[OMNI_WHEEL_LB][OMNI_AXIS_VY] = q;
    ctrl->force_h[OMNI_WHEEL_LB][OMNI_AXIS_WZ] = -lever;
}

static void OmniUpdateCommandToSiScale(OmniVelocityCtrl_s *ctrl)
{
    const float wheel_radius_m = OmniWheelRadiusM();
    const float yaw_lever_m = OmniYawLeverM(ctrl);
    const float degps_to_mps = DEGREE_2_RAD * wheel_radius_m;
    float vx_sum = 0.0f;
    float vy_sum = 0.0f;
    float wz_sum = 0.0f;

    for (uint8_t wheel = 0; wheel < OMNI_WHEEL_NUM; ++wheel) {
        vx_sum += OmniAbs(ctrl->h[wheel][OMNI_AXIS_VX]);
        vy_sum += OmniAbs(ctrl->h[wheel][OMNI_AXIS_VY]);
        wz_sum += OmniAbs(ctrl->h[wheel][OMNI_AXIS_WZ]);
    }

    ctrl->cmd_to_si[OMNI_AXIS_VX] = (vx_sum * 0.25f) * degps_to_mps / OMNI_ROLLING_DIR_Q;
    ctrl->cmd_to_si[OMNI_AXIS_VY] = (vy_sum * 0.25f) * degps_to_mps / OMNI_ROLLING_DIR_Q;
    ctrl->cmd_to_si[OMNI_AXIS_WZ] = (yaw_lever_m > 1.0e-6f) ?
        (wz_sum * 0.25f) * degps_to_mps / yaw_lever_m : 0.0f;

    for (uint8_t axis = 0; axis < OMNI_AXIS_NUM; ++axis) {
        if (ctrl->cmd_to_si[axis] < OMNI_MIN_CMD_TO_SI) {
            ctrl->cmd_to_si[axis] = OMNI_MIN_CMD_TO_SI;
        }
    }
}

static void OmniBuildMatrix(OmniVelocityCtrl_s *ctrl)
{
    /*
     * 普通 X 型 omni 理想速度矩阵。
     * 输入坐标为 vx 前向、vy 左向、wz 逆时针为正；轮序固定为 LF/RF/RB/LB。
     * DJI 电机反馈和参考仍是电机轴单位，底盘层用 268/17 减速比换算到轮端单位。
     * wheel_dir 只修正电机安装方向，不负责减速比换算。
     */
    const float wheel_radius_m = OmniWheelRadiusM();
    const float yaw_lever_m = OmniYawLeverM(ctrl);
    const float wheel_radps_to_degps =
        (wheel_radius_m > 1.0e-6f) ? (1.0f / (DEGREE_2_RAD * wheel_radius_m)) : 0.0f;
    const float vxy_coeff =
        OMNI_ROLLING_DIR_Q * OMNI_IDEAL_CMD_TO_MPS * wheel_radps_to_degps;
    const float wz_coeff =
        yaw_lever_m * OMNI_IDEAL_CMD_TO_RADPS * wheel_radps_to_degps;

    ctrl->h[OMNI_WHEEL_LF][OMNI_AXIS_VX] = vxy_coeff;
    ctrl->h[OMNI_WHEEL_LF][OMNI_AXIS_VY] = -vxy_coeff;
    ctrl->h[OMNI_WHEEL_LF][OMNI_AXIS_WZ] = -wz_coeff;

    ctrl->h[OMNI_WHEEL_RF][OMNI_AXIS_VX] = vxy_coeff;
    ctrl->h[OMNI_WHEEL_RF][OMNI_AXIS_VY] = vxy_coeff;
    ctrl->h[OMNI_WHEEL_RF][OMNI_AXIS_WZ] = wz_coeff;

    ctrl->h[OMNI_WHEEL_RB][OMNI_AXIS_VX] = vxy_coeff;
    ctrl->h[OMNI_WHEEL_RB][OMNI_AXIS_VY] = -vxy_coeff;
    ctrl->h[OMNI_WHEEL_RB][OMNI_AXIS_WZ] = wz_coeff;

    ctrl->h[OMNI_WHEEL_LB][OMNI_AXIS_VX] = vxy_coeff;
    ctrl->h[OMNI_WHEEL_LB][OMNI_AXIS_VY] = vxy_coeff;
    ctrl->h[OMNI_WHEEL_LB][OMNI_AXIS_WZ] = -wz_coeff;

    OmniBuildPhysicalForceMatrix(ctrl);
    OmniUpdateCommandToSiScale(ctrl);
}

static uint8_t OmniSolve3x3(float a[OMNI_AXIS_NUM][OMNI_AXIS_NUM],
                            const float b[OMNI_AXIS_NUM],
                            float x[OMNI_AXIS_NUM])
{
    float m[OMNI_AXIS_NUM][OMNI_AXIS_NUM + 1];

    for (uint8_t r = 0; r < OMNI_AXIS_NUM; ++r) {
        for (uint8_t c = 0; c < OMNI_AXIS_NUM; ++c) {
            m[r][c] = a[r][c];
        }
        m[r][OMNI_AXIS_NUM] = b[r];
    }

    for (uint8_t col = 0; col < OMNI_AXIS_NUM; ++col) {
        uint8_t pivot = col;
        float pivot_abs = OmniAbs(m[col][col]);
        for (uint8_t r = (uint8_t)(col + 1U); r < OMNI_AXIS_NUM; ++r) {
            const float v = OmniAbs(m[r][col]);
            if (v > pivot_abs) {
                pivot = r;
                pivot_abs = v;
            }
        }

        if (pivot_abs < 1.0e-6f) {
            x[0] = 0.0f;
            x[1] = 0.0f;
            x[2] = 0.0f;
            return 0U;
        }

        if (pivot != col) {
            for (uint8_t c = col; c <= OMNI_AXIS_NUM; ++c) {
                const float tmp = m[col][c];
                m[col][c] = m[pivot][c];
                m[pivot][c] = tmp;
            }
        }

        const float inv_pivot = 1.0f / m[col][col];
        for (uint8_t c = col; c <= OMNI_AXIS_NUM; ++c) {
            m[col][c] *= inv_pivot;
        }

        for (uint8_t r = 0; r < OMNI_AXIS_NUM; ++r) {
            if (r == col) {
                continue;
            }
            const float factor = m[r][col];
            for (uint8_t c = col; c <= OMNI_AXIS_NUM; ++c) {
                m[r][c] -= factor * m[col][c];
            }
        }
    }

    x[0] = m[0][OMNI_AXIS_NUM];
    x[1] = m[1][OMNI_AXIS_NUM];
    x[2] = m[2][OMNI_AXIS_NUM];
    return 1U;
}

static void OmniLimitReference(OmniVelocityCtrl_s *ctrl,
                               const OmniVelocityCtrlInput_s *input,
                               float dt)
{
    const float raw_ref[OMNI_AXIS_NUM] = {
        input->vx_ref,
        input->vy_ref,
        input->wz_ref,
    };

    for (uint8_t i = 0; i < OMNI_AXIS_NUM; ++i) {
        const float acc_limit = (i == OMNI_AXIS_WZ) ?
            ctrl->config.wz_acc_limit : ctrl->config.speed_acc_limit;
        const float max_delta = OmniAbs(acc_limit) * dt;
        const float old_ref = ctrl->nu_ref[i];
        const float limited_ref = OmniClamp(raw_ref[i],
                                            old_ref - max_delta,
                                            old_ref + max_delta);
        ctrl->nu_ref_dot[i] = (limited_ref - old_ref) / dt;
        ctrl->nu_ref_last[i] = old_ref;
        ctrl->nu_ref[i] = limited_ref;
    }
}

static void OmniUpdateLqiReference(OmniVelocityCtrl_s *ctrl)
{
    for (uint8_t i = 0; i < OMNI_AXIS_NUM; ++i) {
        ctrl->lqi_ref[i] = ctrl->nu_ref[i] * ctrl->cmd_to_si[i];
        ctrl->lqi_ref_dot[i] = ctrl->nu_ref_dot[i] * ctrl->cmd_to_si[i];
    }
}

static void OmniEstimateVelocity(OmniVelocityCtrl_s *ctrl,
                                 const OmniVelocityCtrlInput_s *input)
{
    float normal[OMNI_AXIS_NUM][OMNI_AXIS_NUM] = {{0.0f}};
    float rhs[OMNI_AXIS_NUM] = {0.0f};
    float solve[OMNI_AXIS_NUM] = {0.0f};

    for (uint8_t wheel = 0; wheel < OMNI_WHEEL_NUM; ++wheel) {
        const float y = ctrl->config.wheel_dir[wheel] * input->wheel_speed[wheel];
        const float weight = 1.0f / (1.0f + 20.0f * ctrl->slip_prob[wheel]);

        for (uint8_t r = 0; r < OMNI_AXIS_NUM; ++r) {
            rhs[r] += weight * ctrl->h[wheel][r] * y;
            for (uint8_t c = 0; c < OMNI_AXIS_NUM; ++c) {
                normal[r][c] += weight * ctrl->h[wheel][r] * ctrl->h[wheel][c];
            }
        }
    }

    for (uint8_t i = 0; i < OMNI_AXIS_NUM; ++i) {
        normal[i][i] += 1.0e-3f;
    }

    if (OmniSolve3x3(normal, rhs, solve)) {
        for (uint8_t i = 0; i < OMNI_AXIS_NUM; ++i) {
            ctrl->nu_hat[i] = solve[i];
        }
    }
}

static void OmniUpdateLqiFeedback(OmniVelocityCtrl_s *ctrl,
                                  const OmniVelocityCtrlInput_s *input)
{
    ctrl->lqi_feedback[OMNI_AXIS_VX] =
        ctrl->nu_hat[OMNI_AXIS_VX] * ctrl->cmd_to_si[OMNI_AXIS_VX];
    ctrl->lqi_feedback[OMNI_AXIS_VY] =
        ctrl->nu_hat[OMNI_AXIS_VY] * ctrl->cmd_to_si[OMNI_AXIS_VY];

    if (input->body_omega_z_valid != 0U) {
        ctrl->lqi_feedback[OMNI_AXIS_WZ] = input->body_omega_z;
        ctrl->lqi_wz_feedback_source = OMNI_LQI_WZ_FEEDBACK_IMU;
    } else {
        ctrl->lqi_feedback[OMNI_AXIS_WZ] =
            ctrl->nu_hat[OMNI_AXIS_WZ] * ctrl->cmd_to_si[OMNI_AXIS_WZ];
        ctrl->lqi_wz_feedback_source = OMNI_LQI_WZ_FEEDBACK_ENCODER;
    }
}

static void OmniUpdateEso(OmniVelocityCtrl_s *ctrl, float dt)
{
    if (ctrl->config.eso_enable == 0U) {
        for (uint8_t i = 0; i < OMNI_AXIS_NUM; ++i) {
            ctrl->nu_eso[i] = ctrl->lqi_feedback[i];
            ctrl->dist_hat[i] = 0.0f;
        }
        return;
    }

    for (uint8_t i = 0; i < OMNI_AXIS_NUM; ++i) {
        const float mass = (OmniAbs(ctrl->config.mass[i]) > 1.0e-6f) ?
            ctrl->config.mass[i] : 1.0f;
        const float w0 = (ctrl->config.eso_bandwidth[i] > 0.0f) ?
            ctrl->config.eso_bandwidth[i] : 0.0f;
        const float beta1 = 2.0f * w0;
        const float beta2 = w0 * w0;
        const float error = ctrl->lqi_feedback[i] - ctrl->nu_eso[i];
        const float model_acc = (ctrl->wrench_act_prev[i] -
                                 ctrl->config.damping[i] * ctrl->nu_eso[i]) / mass;

        ctrl->nu_eso[i] += dt * (model_acc + ctrl->dist_hat[i] + beta1 * error);
        ctrl->dist_hat[i] += dt * beta2 * error;
        ctrl->dist_hat[i] = OmniClampAbs(ctrl->dist_hat[i], 200000.0f);
    }
}

static uint8_t OmniIsEsoCompStaticBlocked(const OmniVelocityCtrl_s *ctrl, uint8_t axis)
{
    const float deadband = (axis == OMNI_AXIS_WZ) ?
        ctrl->config.eso_comp_static_deadband_wz :
        ctrl->config.eso_comp_static_deadband_v;

    if (deadband <= 0.0f) {
        return 0U;
    }

    return (OmniAbs(ctrl->lqi_ref[axis]) < deadband &&
            OmniAbs(ctrl->lqi_feedback[axis]) < deadband) ? 1U : 0U;
}

static uint8_t OmniShouldFreezeIntegral(const OmniVelocityCtrl_s *ctrl,
                                        uint8_t axis,
                                        float err)
{
    const float wrench_err =
        ctrl->wrench_des_prev[axis] - ctrl->wrench_act_prev[axis];
    const float threshold = (axis == OMNI_AXIS_WZ) ?
        OMNI_AW_WRENCH_ERR_TH_WZ_NM : OMNI_AW_WRENCH_ERR_TH_VXY_N;

    if (OmniAbs(wrench_err) <= threshold) {
        return 0U;
    }

    return (err * wrench_err > 0.0f) ? 1U : 0U;
}

static void OmniCalcDesiredWrench(OmniVelocityCtrl_s *ctrl, float dt)
{
    for (uint8_t i = 0; i < OMNI_AXIS_NUM; ++i) {
        const float err = ctrl->lqi_ref[i] - ctrl->lqi_feedback[i];
        const float mass = (OmniAbs(ctrl->config.mass[i]) > 1.0e-6f) ?
            ctrl->config.mass[i] : 1.0f;

        ctrl->lqi_error[i] = err;
        if (OmniAbs(ctrl->config.ki[i]) > 1.0e-6f) {
            if (OmniShouldFreezeIntegral(ctrl, i, err) == 0U) {
                ctrl->integral[i] += err * dt;
            }
            ctrl->integral[i] = OmniClampAbs(ctrl->integral[i], ctrl->config.integral_limit[i]);
        } else {
            ctrl->integral[i] = 0.0f;
        }

        float dist_comp = (ctrl->config.eso_compensate_enable != 0U) ?
            ctrl->config.eso_compensate_gain[i] * ctrl->dist_hat[i] : 0.0f;
        if (OmniIsEsoCompStaticBlocked(ctrl, i) != 0U) {
            dist_comp = 0.0f;
        }
        const float acc_cmd = ctrl->lqi_ref_dot[i] +
                              ctrl->config.kp[i] * err +
                              ctrl->config.ki[i] * ctrl->integral[i] -
                              dist_comp;
        ctrl->wrench_des_prev[i] = mass * acc_cmd +
                                   ctrl->config.damping[i] * ctrl->lqi_feedback[i];
    }
}

static void OmniApplyExternalWrenchFeedforward(OmniVelocityCtrl_s *ctrl,
                                               const OmniVelocityCtrlInput_s *input)
{
    for (uint8_t i = 0; i < OMNI_AXIS_NUM; ++i) {
        const float ff = input->external_wrench_ff[i];

        if (isfinite(ff) != 0) {
            ctrl->wrench_des_prev[i] += ff;
        }
    }
}

static void OmniApplyMultiSlipWrenchDerate(OmniVelocityCtrl_s *ctrl)
{
    float p_max = 0.0f;
    float p_sum = 0.0f;

    for (uint8_t wheel = 0; wheel < OMNI_WHEEL_NUM; ++wheel) {
        const float p = OmniClamp(ctrl->slip_prob[wheel], 0.0f, 1.0f);
        p_sum += p;
        if (p > p_max) {
            p_max = p;
        }
    }

    const float p_avg = p_sum * 0.25f;
    const uint8_t avg_triggered = (p_avg > OMNI_MULTI_SLIP_AVG_START) ? 1U : 0U;
    const uint8_t max_triggered =
        (p_max > OMNI_MULTI_SLIP_MAX_TRIGGER && p_avg > OMNI_MULTI_SLIP_AVG_TRIGGER) ? 1U : 0U;

    if (avg_triggered == 0U && max_triggered == 0U) {
        return;
    }

    const float denom = OMNI_MULTI_SLIP_AVG_FULL - OMNI_MULTI_SLIP_AVG_START;
    const float ratio = (denom > 1.0e-6f) ?
        OmniClamp((p_avg - OMNI_MULTI_SLIP_AVG_START) / denom, 0.0f, 1.0f) : 1.0f;
    const float scale = 1.0f -
        (1.0f - OMNI_MULTI_SLIP_MIN_WRENCH_SCALE) * ratio;

    for (uint8_t axis = 0; axis < OMNI_AXIS_NUM; ++axis) {
        ctrl->wrench_des_prev[axis] *= scale;
    }
}

static float OmniGetSlipForceWeight(const OmniVelocityCtrl_s *ctrl, uint8_t wheel);

static void OmniSolveFreeForces(const OmniVelocityCtrl_s *ctrl,
                                const uint8_t free_mask[OMNI_WHEEL_NUM],
                                const float wrench_rem[OMNI_AXIS_NUM],
                                float force[OMNI_WHEEL_NUM])
{
    float aat[OMNI_AXIS_NUM][OMNI_AXIS_NUM] = {{0.0f}};
    float lambda[OMNI_AXIS_NUM] = {0.0f};

    for (uint8_t wheel = 0; wheel < OMNI_WHEEL_NUM; ++wheel) {
        const float weight = OmniGetSlipForceWeight(ctrl, wheel);

        if (free_mask[wheel] == 0U) {
            continue;
        }
        for (uint8_t r = 0; r < OMNI_AXIS_NUM; ++r) {
            for (uint8_t c = 0; c < OMNI_AXIS_NUM; ++c) {
                aat[r][c] += weight * ctrl->force_h[wheel][r] * ctrl->force_h[wheel][c];
            }
        }
    }

    for (uint8_t i = 0; i < OMNI_AXIS_NUM; ++i) {
        aat[i][i] += ctrl->config.allocator_regularization;
    }

    (void)OmniSolve3x3(aat, wrench_rem, lambda);

    for (uint8_t wheel = 0; wheel < OMNI_WHEEL_NUM; ++wheel) {
        const float weight = OmniGetSlipForceWeight(ctrl, wheel);
        if (free_mask[wheel] == 0U) {
            continue;
        }
        float f = 0.0f;
        for (uint8_t axis = 0; axis < OMNI_AXIS_NUM; ++axis) {
            f += ctrl->force_h[wheel][axis] * lambda[axis];
        }
        force[wheel] = weight * f;
    }
}

static float OmniGetSlipForceWeight(const OmniVelocityCtrl_s *ctrl, uint8_t wheel)
{
    const float gain = (ctrl->config.slip_force_gain > 0.0f) ?
        ctrl->config.slip_force_gain : 0.0f;
    const float min_weight = OmniClamp(ctrl->config.slip_force_min_weight, 0.0f, 1.0f);
    const float slip_prob = OmniClamp(ctrl->slip_prob[wheel], 0.0f, 1.0f);
    const float weight = (1.0f - slip_prob) / (1.0f + gain * slip_prob);

    if (ctrl->config.slip_force_enable == 0U || gain <= 0.0f) {
        return 1.0f;
    }
    if (slip_prob >= 1.0f) {
        return min_weight;
    }
    return OmniClamp(weight, min_weight, 1.0f);
}

static float OmniForceToCurrentFit(const OmniVelocityCtrl_s *ctrl,
                                   uint8_t wheel,
                                   float force,
                                   uint8_t offset_enable)
{
    const float k = (force >= 0.0f) ?
        ctrl->config.force_to_cmd_pos[wheel] : ctrl->config.force_to_cmd_neg[wheel];
    float current = k * force;

    if (offset_enable != 0U && OmniAbs(force) < OMNI_CURRENT_OFFSET_FORCE_DEADBAND_N) {
        return 0.0f;
    }

    if (offset_enable != 0U && OmniAbs(force) > 1.0e-6f) {
        current += ctrl->config.current_offset[wheel] * OmniSignNonZero(force);
    }
    return current;
}

static float OmniCurrentFitToForce(const OmniVelocityCtrl_s *ctrl,
                                   uint8_t wheel,
                                   float current,
                                   uint8_t offset_enable)
{
    const float sign = OmniSignNonZero(current);
    float current_eff = current;
    float k;

    if (OmniAbs(current) < 1.0e-6f) {
        return 0.0f;
    }

    if (offset_enable != 0U) {
        const float offset = ctrl->config.current_offset[wheel];
        current_eff = current - offset * sign;
        if (current_eff * sign < 0.0f) {
            return 0.0f;
        }
    }

    k = (current_eff >= 0.0f) ?
        ctrl->config.force_to_cmd_pos[wheel] : ctrl->config.force_to_cmd_neg[wheel];
    if (OmniAbs(k) < 1.0e-6f) {
        return 0.0f;
    }
    return current_eff / k;
}

static void OmniBuildForceBounds(const OmniVelocityCtrl_s *ctrl,
                                 const OmniVelocityCtrlInput_s *input,
                                 float force_min[OMNI_WHEEL_NUM],
                                 float force_max[OMNI_WHEEL_NUM])
{
    const float dt = OmniSanitizeDt(ctrl, input->dt);
    const float rate_limit = input->current_rate_limit;

    for (uint8_t wheel = 0; wheel < OMNI_WHEEL_NUM; ++wheel) {
        float current_limit = input->current_limit;
        float pos_k = ctrl->config.force_to_cmd_pos[wheel];
        float neg_k = ctrl->config.force_to_cmd_neg[wheel];
        float current_min;
        float current_max;

        if (current_limit <= 0.0f) {
            current_limit = ctrl->config.wheel_current_limit[wheel];
        }
        if (current_limit <= 0.0f) {
            current_limit = OMNI_WHEEL_FORCE_LIMIT * pos_k;
        }
        if (pos_k <= 1.0e-6f) {
            pos_k = 1.0f;
        }
        if (neg_k <= 1.0e-6f) {
            neg_k = pos_k;
        }

        current_min = -OmniAbs(current_limit);
        current_max = OmniAbs(current_limit);
        if (rate_limit > 0.0f) {
            const float max_delta = OmniAbs(rate_limit) * dt;
            const float current_prev = ctrl->current_ref_fit_prev[wheel];
            const float rate_min = current_prev - max_delta;
            const float rate_max = current_prev + max_delta;

            if (rate_min > current_min) {
                current_min = rate_min;
            }
            if (rate_max < current_max) {
                current_max = rate_max;
            }
            if (current_min > current_max) {
                const float current_target =
                    OmniClamp(current_prev, -OmniAbs(current_limit), OmniAbs(current_limit));
                current_min = current_target;
                current_max = current_target;
            }
        }

        force_min[wheel] = OmniCurrentFitToForce(ctrl,
                                                 wheel,
                                                 current_min,
                                                 input->current_offset_enable);
        force_max[wheel] = OmniCurrentFitToForce(ctrl,
                                                 wheel,
                                                 current_max,
                                                 input->current_offset_enable);
        if (force_min[wheel] > force_max[wheel]) {
            const float tmp = force_min[wheel];
            force_min[wheel] = force_max[wheel];
            force_max[wheel] = tmp;
        }
    }
}

static void OmniAllocateForceForWrench(const OmniVelocityCtrl_s *ctrl,
                                       const OmniVelocityCtrlInput_s *input,
                                       const float wrench_des[OMNI_AXIS_NUM],
                                       float force_cmd[OMNI_WHEEL_NUM],
                                       uint8_t *limit_active)
{
    uint8_t free_mask[OMNI_WHEEL_NUM] = {1U, 1U, 1U, 1U};
    float force_min[OMNI_WHEEL_NUM];
    float force_max[OMNI_WHEEL_NUM];

    for (uint8_t i = 0; i < OMNI_WHEEL_NUM; ++i) {
        force_cmd[i] = 0.0f;
    }
    OmniBuildForceBounds(ctrl, input, force_min, force_max);

    for (uint8_t iter = 0; iter < OMNI_WHEEL_NUM; ++iter) {
        float wrench_rem[OMNI_AXIS_NUM];
        float max_violation = 0.0f;
        int8_t clamp_wheel = -1;
        float clamp_value = 0.0f;

        for (uint8_t axis = 0; axis < OMNI_AXIS_NUM; ++axis) {
            wrench_rem[axis] = wrench_des[axis];
            for (uint8_t wheel = 0; wheel < OMNI_WHEEL_NUM; ++wheel) {
                if (free_mask[wheel] == 0U) {
                    wrench_rem[axis] -= ctrl->force_h[wheel][axis] * force_cmd[wheel];
                }
            }
        }

        OmniSolveFreeForces(ctrl, free_mask, wrench_rem, force_cmd);

        for (uint8_t wheel = 0; wheel < OMNI_WHEEL_NUM; ++wheel) {
            if (free_mask[wheel] == 0U) {
                continue;
            }
            if (force_cmd[wheel] > force_max[wheel]) {
                const float violation = force_cmd[wheel] - force_max[wheel];
                if (violation > max_violation) {
                    max_violation = violation;
                    clamp_wheel = (int8_t)wheel;
                    clamp_value = force_max[wheel];
                }
            } else if (force_cmd[wheel] < force_min[wheel]) {
                const float violation = force_min[wheel] - force_cmd[wheel];
                if (violation > max_violation) {
                    max_violation = violation;
                    clamp_wheel = (int8_t)wheel;
                    clamp_value = force_min[wheel];
                }
            }
        }

        if (clamp_wheel < 0) {
            break;
        }

        force_cmd[(uint8_t)clamp_wheel] = clamp_value;
        free_mask[(uint8_t)clamp_wheel] = 0U;
        *limit_active = 1U;
    }

    for (uint8_t wheel = 0; wheel < OMNI_WHEEL_NUM; ++wheel) {
        force_cmd[wheel] = OmniClamp(force_cmd[wheel], force_min[wheel], force_max[wheel]);
    }
}

static void OmniAllocateForce(const OmniVelocityCtrl_s *ctrl,
                              const OmniVelocityCtrlInput_s *input,
                              float force_cmd[OMNI_WHEEL_NUM],
                              uint8_t *limit_active)
{
    OmniAllocateForceForWrench(ctrl, input, ctrl->wrench_des_prev, force_cmd, limit_active);
}

static uint8_t OmniIsEsoCompStaticBlockedForRef(const OmniVelocityCtrl_s *ctrl,
                                                uint8_t axis,
                                                float lqi_ref)
{
    const float deadband = (axis == OMNI_AXIS_WZ) ?
        ctrl->config.eso_comp_static_deadband_wz :
        ctrl->config.eso_comp_static_deadband_v;

    if (deadband <= 0.0f) {
        return 0U;
    }
    return (OmniAbs(lqi_ref) < deadband &&
            OmniAbs(ctrl->lqi_feedback[axis]) < deadband) ? 1U : 0U;
}

static void OmniPredictWrenchForRef(const OmniVelocityCtrl_s *ctrl,
                                    const OmniVelocityCtrlInput_s *input,
                                    const float ref[OMNI_AXIS_NUM],
                                    const float ref_dot[OMNI_AXIS_NUM],
                                    float wrench[OMNI_AXIS_NUM])
{
    for (uint8_t axis = 0; axis < OMNI_AXIS_NUM; ++axis) {
        const float lqi_ref = ref[axis] * ctrl->cmd_to_si[axis];
        const float lqi_ref_dot = ref_dot[axis] * ctrl->cmd_to_si[axis];
        const float err = lqi_ref - ctrl->lqi_feedback[axis];
        const float mass = (OmniAbs(ctrl->config.mass[axis]) > 1.0e-6f) ?
            ctrl->config.mass[axis] : 1.0f;
        float dist_comp = (ctrl->config.eso_compensate_enable != 0U) ?
            ctrl->config.eso_compensate_gain[axis] * ctrl->dist_hat[axis] : 0.0f;
        const float ff = input->external_wrench_ff[axis];

        if (OmniIsEsoCompStaticBlockedForRef(ctrl, axis, lqi_ref) != 0U) {
            dist_comp = 0.0f;
        }

        const float acc_cmd = lqi_ref_dot +
                              ctrl->config.kp[axis] * err +
                              ctrl->config.ki[axis] * ctrl->integral[axis] -
                              dist_comp;
        wrench[axis] = mass * acc_cmd +
                       ctrl->config.damping[axis] * ctrl->lqi_feedback[axis];
        if (isfinite(ff) != 0) {
            wrench[axis] += ff;
        }
    }
}

static void OmniBuildWheelSpeedFitForRef(const OmniVelocityCtrl_s *ctrl,
                                         const float ref[OMNI_AXIS_NUM],
                                         float wheel_speed_fit[OMNI_WHEEL_NUM])
{
    for (uint8_t wheel = 0; wheel < OMNI_WHEEL_NUM; ++wheel) {
        float speed_fit = 0.0f;

        for (uint8_t axis = 0; axis < OMNI_AXIS_NUM; ++axis) {
            speed_fit += ctrl->h[wheel][axis] * ref[axis];
        }
        wheel_speed_fit[wheel] = speed_fit;
    }
}

static float OmniSanitizeSearchScale(float scale, float fallback)
{
    if (isfinite(scale) == 0) {
        scale = fallback;
    }
    if (scale < 0.0f) {
        scale = 0.0f;
    }
    if (scale > OMNI_ROTATE_SEARCH_SCALE_MAX) {
        scale = OMNI_ROTATE_SEARCH_SCALE_MAX;
    }
    return scale;
}

static void OmniBuildRefForScale(const float base_ref[OMNI_AXIS_NUM],
                                 float sv,
                                 float sw,
                                 float ref[OMNI_AXIS_NUM])
{
    sv = OmniSanitizeSearchScale(sv, 0.0f);
    sw = OmniClamp(OmniSanitizeSearchScale(sw, 0.0f), 0.0f, 1.0f);

    ref[OMNI_AXIS_VX] = base_ref[OMNI_AXIS_VX] * sv;
    ref[OMNI_AXIS_VY] = base_ref[OMNI_AXIS_VY] * sv;
    ref[OMNI_AXIS_WZ] = base_ref[OMNI_AXIS_WZ] * sw;
}

static float OmniRotateWzSearchSign(const OmniVelocityCtrlInput_s *input,
                                    float ramped_wz_ref)
{
    if (input != 0 &&
        isfinite(input->wz_ref) != 0 &&
        OmniAbs(input->wz_ref) > OMNI_ROTATE_VXY_ACTIVE_EPS) {
        return (input->wz_ref < 0.0f) ? -1.0f : 1.0f;
    }
    if (isfinite(ramped_wz_ref) != 0 &&
        OmniAbs(ramped_wz_ref) > OMNI_ROTATE_VXY_ACTIVE_EPS) {
        return (ramped_wz_ref < 0.0f) ? -1.0f : 1.0f;
    }
    return 1.0f;
}

static float OmniRotateWzSearchAbsLimit(const OmniVelocityCtrlInput_s *input,
                                        float ramped_wz_ref)
{
    float limit = 0.0f;

    if (input != 0 && isfinite(input->rotate_wz_search_limit) != 0) {
        limit = OmniAbs(input->rotate_wz_search_limit);
    }
    if (limit <= OMNI_ROTATE_VXY_ACTIVE_EPS &&
        input != 0 &&
        isfinite(input->wz_ref) != 0) {
        limit = OmniAbs(input->wz_ref);
    }
    if (limit <= OMNI_ROTATE_VXY_ACTIVE_EPS &&
        isfinite(ramped_wz_ref) != 0) {
        limit = OmniAbs(ramped_wz_ref);
    }
    return limit;
}

static void OmniApplyRotateWzSearchLimit(const OmniVelocityCtrlInput_s *input,
                                         float base_ref[OMNI_AXIS_NUM])
{
    const float wz_limit =
        OmniRotateWzSearchAbsLimit(input, base_ref[OMNI_AXIS_WZ]);

    if (wz_limit <= OMNI_ROTATE_VXY_ACTIVE_EPS) {
        return;
    }

    base_ref[OMNI_AXIS_WZ] =
        OmniRotateWzSearchSign(input, base_ref[OMNI_AXIS_WZ]) * wz_limit;
}

static float OmniMaxAbsWheelSpeedForRef(const OmniVelocityCtrl_s *ctrl,
                                        const float ref[OMNI_AXIS_NUM])
{
    float wheel_speed_fit[OMNI_WHEEL_NUM];
    float max_abs_speed = 0.0f;

    OmniBuildWheelSpeedFitForRef(ctrl, ref, wheel_speed_fit);
    for (uint8_t wheel = 0U; wheel < OMNI_WHEEL_NUM; ++wheel) {
        const float abs_speed = OmniAbs(wheel_speed_fit[wheel]);

        if (abs_speed > max_abs_speed) {
            max_abs_speed = abs_speed;
        }
    }
    return max_abs_speed;
}

static float OmniPowerWheelSpeedLimit(const OmniVelocityCtrl_s *ctrl)
{
    const float limit = ctrl->config.power_wheel_speed_limit;

    return (limit > 0.0f && isfinite(limit) != 0) ?
        limit : OMNI_POWER_WHEEL_SPEED_LIMIT_DEFAULT;
}

static float OmniVxyMagnitude(const float ref[OMNI_AXIS_NUM])
{
    return sqrtf(ref[OMNI_AXIS_VX] * ref[OMNI_AXIS_VX] +
                 ref[OMNI_AXIS_VY] * ref[OMNI_AXIS_VY]);
}

static uint8_t OmniBuildVxyDir(const float ref[OMNI_AXIS_NUM],
                               float *mag,
                               float *dir_x,
                               float *dir_y)
{
    const float vxy = OmniVxyMagnitude(ref);

    if (mag != 0) {
        *mag = vxy;
    }
    if (vxy <= OMNI_ROTATE_VXY_ACTIVE_EPS || isfinite(vxy) == 0) {
        if (dir_x != 0) {
            *dir_x = 0.0f;
        }
        if (dir_y != 0) {
            *dir_y = 0.0f;
        }
        return 0U;
    }

    if (dir_x != 0) {
        *dir_x = ref[OMNI_AXIS_VX] / vxy;
    }
    if (dir_y != 0) {
        *dir_y = ref[OMNI_AXIS_VY] / vxy;
    }
    return 1U;
}

static uint8_t OmniScalarMatches(float cached, float current, float abs_tol, float rel_tol)
{
    float tol;

    if (isfinite(cached) == 0 || isfinite(current) == 0 || cached <= 0.0f || current <= 0.0f) {
        return 0U;
    }

    tol = abs_tol + rel_tol * OmniAbs(current);
    return (OmniAbs(cached - current) <= tol) ? 1U : 0U;
}

static void OmniClearReachableCacheState(OmniVelocityCtrl_s *ctrl)
{
    ctrl->reachable_vxy_valid = 0U;
    ctrl->reachable_wz_valid = 0U;
    ctrl->rotate_strategy_active = 0U;
    ctrl->reachable_vxy_search_success = 0U;
    ctrl->reachable_wz_search_success = 0U;
    ctrl->reachable_vxy_max = 0.0f;
    ctrl->reachable_wz_max = 0.0f;
    ctrl->reachable_vxy_dir_x = 0.0f;
    ctrl->reachable_vxy_dir_y = 0.0f;
    ctrl->reachable_vxy_budget_w = 0.0f;
    ctrl->reachable_wz_budget_w = 0.0f;
    ctrl->reachable_vxy_wheel_speed_limit = 0.0f;
    ctrl->reachable_wz_wheel_speed_limit = 0.0f;
}

static void OmniStoreReachableVxy(OmniVelocityCtrl_s *ctrl,
                                  float vxy_max,
                                  float dir_x,
                                  float dir_y,
                                  float budget_w,
                                  float wheel_speed_limit)
{
    ctrl->reachable_vxy_valid =
        (isfinite(vxy_max) != 0 &&
         isfinite(dir_x) != 0 &&
         isfinite(dir_y) != 0 &&
         isfinite(budget_w) != 0 &&
         isfinite(wheel_speed_limit) != 0 &&
         vxy_max > OMNI_ROTATE_VXY_ACTIVE_EPS &&
         budget_w > 0.0f &&
         wheel_speed_limit > 0.0f) ? 1U : 0U;
    ctrl->reachable_vxy_max = (ctrl->reachable_vxy_valid != 0U) ? vxy_max : 0.0f;
    ctrl->reachable_vxy_dir_x = (ctrl->reachable_vxy_valid != 0U) ? dir_x : 0.0f;
    ctrl->reachable_vxy_dir_y = (ctrl->reachable_vxy_valid != 0U) ? dir_y : 0.0f;
    ctrl->reachable_vxy_budget_w = (ctrl->reachable_vxy_valid != 0U) ? budget_w : 0.0f;
    ctrl->reachable_vxy_wheel_speed_limit =
        (ctrl->reachable_vxy_valid != 0U) ? wheel_speed_limit : 0.0f;
}

static void OmniStoreReachableWz(OmniVelocityCtrl_s *ctrl,
                                 float wz_max,
                                 float budget_w,
                                 float wheel_speed_limit)
{
    ctrl->reachable_wz_valid =
        (isfinite(wz_max) != 0 &&
         isfinite(budget_w) != 0 &&
         isfinite(wheel_speed_limit) != 0 &&
         wz_max > OMNI_ROTATE_VXY_ACTIVE_EPS &&
         budget_w > 0.0f &&
         wheel_speed_limit > 0.0f) ? 1U : 0U;
    ctrl->reachable_wz_max = (ctrl->reachable_wz_valid != 0U) ? wz_max : 0.0f;
    ctrl->reachable_wz_budget_w = (ctrl->reachable_wz_valid != 0U) ? budget_w : 0.0f;
    ctrl->reachable_wz_wheel_speed_limit =
        (ctrl->reachable_wz_valid != 0U) ? wheel_speed_limit : 0.0f;
}

static void OmniApplyReferenceScale(OmniVelocityCtrl_s *ctrl,
                                    const float base_ref[OMNI_AXIS_NUM],
                                    float sv,
                                    float sw,
                                    float dt)
{
    ctrl->power_vxy_scale = OmniClamp(sv, 0.0f, 1.0f);
    ctrl->power_wz_scale = OmniClamp(sw, 0.0f, 1.0f);

    ctrl->nu_ref[OMNI_AXIS_VX] = base_ref[OMNI_AXIS_VX] * ctrl->power_vxy_scale;
    ctrl->nu_ref[OMNI_AXIS_VY] = base_ref[OMNI_AXIS_VY] * ctrl->power_vxy_scale;
    ctrl->nu_ref[OMNI_AXIS_WZ] = base_ref[OMNI_AXIS_WZ] * ctrl->power_wz_scale;

    for (uint8_t axis = 0; axis < OMNI_AXIS_NUM; ++axis) {
        ctrl->nu_ref_dot[axis] = (ctrl->nu_ref[axis] - ctrl->nu_ref_last[axis]) / dt;
    }
}

static uint8_t OmniReachableVxyMatches(const OmniVelocityCtrl_s *ctrl,
                                       float dir_x,
                                       float dir_y,
                                       float budget_w,
                                       float wheel_speed_limit)
{
    const float dot = ctrl->reachable_vxy_dir_x * dir_x +
                      ctrl->reachable_vxy_dir_y * dir_y;

    if (ctrl->reachable_vxy_valid == 0U ||
        dot < OMNI_REACHABLE_DIR_DOT_MIN ||
        OmniScalarMatches(ctrl->reachable_vxy_budget_w,
                          budget_w,
                          OMNI_REACHABLE_BUDGET_ABS_TOL_W,
                          OMNI_REACHABLE_BUDGET_REL_TOL) == 0U ||
        OmniScalarMatches(ctrl->reachable_vxy_wheel_speed_limit,
                          wheel_speed_limit,
                          OMNI_POWER_SCALE_EPS,
                          OMNI_REACHABLE_BUDGET_REL_TOL) == 0U) {
        return 0U;
    }
    return 1U;
}

static uint8_t OmniReachableWzMatches(const OmniVelocityCtrl_s *ctrl,
                                      float budget_w,
                                      float wheel_speed_limit)
{
    if (ctrl->reachable_wz_valid == 0U ||
        OmniScalarMatches(ctrl->reachable_wz_budget_w,
                          budget_w,
                          OMNI_REACHABLE_BUDGET_ABS_TOL_W,
                          OMNI_REACHABLE_BUDGET_REL_TOL) == 0U ||
        OmniScalarMatches(ctrl->reachable_wz_wheel_speed_limit,
                          wheel_speed_limit,
                          OMNI_POWER_SCALE_EPS,
                          OMNI_REACHABLE_BUDGET_REL_TOL) == 0U) {
        return 0U;
    }
    return 1U;
}

static float OmniEstimateSpeedLoopCurrentFit(const OmniVelocityCtrl_s *ctrl,
                                             const OmniVelocityCtrlInput_s *input,
                                             uint8_t wheel,
                                             float wheel_speed_ref_fit,
                                             float force_cmd)
{
    const float ratio = (input->wheel_to_motor_speed_ratio > 1.0e-6f &&
                         isfinite(input->wheel_to_motor_speed_ratio) != 0) ?
        input->wheel_to_motor_speed_ratio : 1.0f;
    const float motor_ref =
        ctrl->config.wheel_dir[wheel] * wheel_speed_ref_fit * ratio;
    const float motor_feedback = input->wheel_speed[wheel] * ratio;
    const float err = motor_ref - motor_feedback;
    const float kp = (isfinite(input->speed_power_kp[wheel]) != 0) ?
        input->speed_power_kp[wheel] : 0.0f;
    const float iout = (isfinite(input->speed_power_iout[wheel]) != 0) ?
        input->speed_power_iout[wheel] : 0.0f;
    const float deadband = (input->speed_power_deadband[wheel] > 0.0f &&
                            isfinite(input->speed_power_deadband[wheel]) != 0) ?
        input->speed_power_deadband[wheel] : 0.0f;
    const float output_limit = (input->speed_power_output_limit[wheel] > 0.0f &&
                                isfinite(input->speed_power_output_limit[wheel]) != 0) ?
        input->speed_power_output_limit[wheel] : 16384.0f;
    float speed_pid_current_motor = 0.0f;
    const float ff_current_fit = OmniForceToCurrentFit(ctrl,
                                                       wheel,
                                                       force_cmd,
                                                       input->current_offset_enable);
    const float ff_current_motor = ctrl->config.wheel_dir[wheel] * ff_current_fit;

    if (OmniAbs(err) > deadband) {
        speed_pid_current_motor = kp * err + iout;
        speed_pid_current_motor = OmniClampAbs(speed_pid_current_motor, output_limit);
    }

    return ctrl->config.wheel_dir[wheel] *
        (speed_pid_current_motor + ff_current_motor);
}

static float OmniEstimatePositiveWheelPower(const OmniVelocityCtrl_s *ctrl,
                                            const OmniVelocityCtrlInput_s *input,
                                            const float force_cmd[OMNI_WHEEL_NUM],
                                            const float ref[OMNI_AXIS_NUM])
{
    const float wheel_radius_m = OmniWheelRadiusM();
    float wheel_speed_ref_fit[OMNI_WHEEL_NUM] = {0.0f};
    float power_w = 0.0f;

    if (input->speed_power_model_enable != 0U) {
        OmniBuildWheelSpeedFitForRef(ctrl, ref, wheel_speed_ref_fit);
    }

    for (uint8_t wheel = 0; wheel < OMNI_WHEEL_NUM; ++wheel) {
        float force_for_power = force_cmd[wheel];
        float wheel_speed_fit;

        if (input->speed_power_model_enable != 0U) {
            const float current_fit =
                OmniEstimateSpeedLoopCurrentFit(ctrl,
                                                input,
                                                wheel,
                                                wheel_speed_ref_fit[wheel],
                                                force_cmd[wheel]);
            force_for_power = OmniCurrentFitToForce(ctrl,
                                                    wheel,
                                                    current_fit,
                                                    input->current_offset_enable);
            wheel_speed_fit = wheel_speed_ref_fit[wheel];
        } else {
            wheel_speed_fit = ctrl->config.wheel_dir[wheel] * input->wheel_speed[wheel];
        }

        const float wheel_speed_mps = wheel_speed_fit * DEGREE_2_RAD * wheel_radius_m;
        const float wheel_power_w = force_for_power * wheel_speed_mps;

        if (isfinite(wheel_power_w) != 0 && wheel_power_w > 0.0f) {
            power_w += wheel_power_w;
        }
    }
    return power_w;
}

static float OmniEstimatePowerForScale(const OmniVelocityCtrl_s *ctrl,
                                       const OmniVelocityCtrlInput_s *input,
                                       const float base_ref[OMNI_AXIS_NUM],
                                       float sv,
                                       float sw,
                                       float dt)
{
    float ref[OMNI_AXIS_NUM];
    float ref_dot[OMNI_AXIS_NUM];
    float wrench[OMNI_AXIS_NUM];
    float force_cmd[OMNI_WHEEL_NUM];
    uint8_t limit_active = 0U;

    OmniBuildRefForScale(base_ref, sv, sw, ref);

    for (uint8_t axis = 0; axis < OMNI_AXIS_NUM; ++axis) {
        ref_dot[axis] = (ref[axis] - ctrl->nu_ref_last[axis]) / dt;
    }

    OmniPredictWrenchForRef(ctrl, input, ref, ref_dot, wrench);
    OmniAllocateForceForWrench(ctrl, input, wrench, force_cmd, &limit_active);
    return OmniEstimatePositiveWheelPower(ctrl, input, force_cmd, ref);
}

static uint8_t OmniScaleFitsSearchBudget(const OmniVelocityCtrl_s *ctrl,
                                         const OmniVelocityCtrlInput_s *input,
                                         const float base_ref[OMNI_AXIS_NUM],
                                         float sv,
                                         float sw,
                                         float budget_w,
                                         float wheel_speed_limit,
                                         float dt,
                                         float *power_w)
{
    float ref[OMNI_AXIS_NUM];
    const float power =
        OmniEstimatePowerForScale(ctrl, input, base_ref, sv, sw, dt);

    if (power_w != 0) {
        *power_w = power;
    }
    if (isfinite(budget_w) == 0 ||
        budget_w <= 0.0f ||
        isfinite(power) == 0 ||
        power > budget_w + OMNI_POWER_SCALE_EPS) {
        return 0U;
    }

    if (wheel_speed_limit > 0.0f && isfinite(wheel_speed_limit) != 0) {
        OmniBuildRefForScale(base_ref, sv, sw, ref);
        if (OmniMaxAbsWheelSpeedForRef(ctrl, ref) >
            wheel_speed_limit + OMNI_POWER_SCALE_EPS) {
            return 0U;
        }
    }

    return 1U;
}

static float OmniFindPureVxyScaleHigh(const OmniVelocityCtrl_s *ctrl,
                                      const float base_ref[OMNI_AXIS_NUM],
                                      float wheel_speed_limit)
{
    float max_abs_speed_per_scale = 0.0f;

    if (wheel_speed_limit <= 0.0f || isfinite(wheel_speed_limit) == 0) {
        return 1.0f;
    }

    for (uint8_t wheel = 0U; wheel < OMNI_WHEEL_NUM; ++wheel) {
        const float speed_per_scale =
            ctrl->h[wheel][OMNI_AXIS_VX] * base_ref[OMNI_AXIS_VX] +
            ctrl->h[wheel][OMNI_AXIS_VY] * base_ref[OMNI_AXIS_VY];
        const float abs_speed = OmniAbs(speed_per_scale);

        if (abs_speed > max_abs_speed_per_scale) {
            max_abs_speed_per_scale = abs_speed;
        }
    }

    if (max_abs_speed_per_scale <= OMNI_POWER_SCALE_EPS) {
        return 1.0f;
    }

    return OmniSanitizeSearchScale(wheel_speed_limit / max_abs_speed_per_scale,
                                   1.0f);
}

static float OmniFindMaxSvForPower(const OmniVelocityCtrl_s *ctrl,
                                   const OmniVelocityCtrlInput_s *input,
                                   const float base_ref[OMNI_AXIS_NUM],
                                   float sw,
                                   float low,
                                   float high,
                                   float budget_w,
                                   float wheel_speed_limit,
                                   float dt)
{
    low = OmniSanitizeSearchScale(low, 0.0f);
    high = OmniSanitizeSearchScale(high, low);
    if (high < low) {
        high = low;
    }

    if (OmniScaleFitsSearchBudget(ctrl,
                                  input,
                                  base_ref,
                                  low,
                                  sw,
                                  budget_w,
                                  wheel_speed_limit,
                                  dt,
                                  0) == 0U) {
        return low;
    }
    if (OmniScaleFitsSearchBudget(ctrl,
                                  input,
                                  base_ref,
                                  high,
                                  sw,
                                  budget_w,
                                  wheel_speed_limit,
                                  dt,
                                  0) != 0U) {
        return high;
    }

    for (uint8_t i = 0U; i < OMNI_POWER_SCALE_BISECT_STEPS; ++i) {
        const float mid = 0.5f * (low + high);

        if (OmniScaleFitsSearchBudget(ctrl,
                                      input,
                                      base_ref,
                                      mid,
                                      sw,
                                      budget_w,
                                      wheel_speed_limit,
                                      dt,
                                      0) != 0U) {
            low = mid;
        } else {
            high = mid;
        }
    }
    return low;
}

static float OmniFindMaxSwForPower(const OmniVelocityCtrl_s *ctrl,
                                   const OmniVelocityCtrlInput_s *input,
                                   const float base_ref[OMNI_AXIS_NUM],
                                   float sv,
                                   float high,
                                   float budget_w,
                                   float wheel_speed_limit,
                                   float dt)
{
    float low = 0.0f;
    high = OmniClamp(OmniSanitizeSearchScale(high, 1.0f), 0.0f, 1.0f);

    if (OmniScaleFitsSearchBudget(ctrl,
                                  input,
                                  base_ref,
                                  sv,
                                  low,
                                  budget_w,
                                  wheel_speed_limit,
                                  dt,
                                  0) == 0U) {
        return 0.0f;
    }
    if (OmniScaleFitsSearchBudget(ctrl,
                                  input,
                                  base_ref,
                                  sv,
                                  high,
                                  budget_w,
                                  wheel_speed_limit,
                                  dt,
                                  0) != 0U) {
        return high;
    }

    for (uint8_t i = 0U; i < OMNI_POWER_SCALE_BISECT_STEPS; ++i) {
        const float mid = 0.5f * (low + high);

        if (OmniScaleFitsSearchBudget(ctrl,
                                      input,
                                      base_ref,
                                      sv,
                                      mid,
                                      budget_w,
                                      wheel_speed_limit,
                                      dt,
                                      0) != 0U) {
            low = mid;
        } else {
            high = mid;
        }
    }
    return low;
}

static uint8_t OmniMeasuredPowerValid(const OmniVelocityCtrlInput_s *input)
{
    if (input == 0 ||
        input->measured_power_valid == 0U ||
        isfinite(input->measured_power_w) == 0 ||
        input->measured_power_w <= 0.0f) {
        return 0U;
    }
    return 1U;
}

static void OmniUpdatePowerFeedback(OmniVelocityCtrl_s *ctrl,
                                    const OmniVelocityCtrlInput_s *input,
                                    float dt)
{
    ctrl->power_real_valid = OmniMeasuredPowerValid(input);
    ctrl->power_real_w = (ctrl->power_real_valid != 0U) ? input->measured_power_w : 0.0f;

    if (ctrl->power_real_valid != 0U) {
        const float tau_s =
            (ctrl->config.power_feedback_lpf_tau_s > 0.0f &&
             isfinite(ctrl->config.power_feedback_lpf_tau_s) != 0) ?
            ctrl->config.power_feedback_lpf_tau_s :
            OMNI_POWER_FEEDBACK_DEFAULT_LPF_TAU_S;
        const float alpha = OmniClamp(dt / (tau_s + dt), 0.0f, 1.0f);

        if (ctrl->power_real_lpf_valid == 0U ||
            isfinite(ctrl->power_real_lpf_w) == 0) {
            ctrl->power_real_lpf_w = ctrl->power_real_w;
            ctrl->power_real_lpf_valid = 1U;
        } else {
            ctrl->power_real_lpf_w += alpha * (ctrl->power_real_w - ctrl->power_real_lpf_w);
        }
    } else {
        ctrl->power_real_lpf_w = 0.0f;
        ctrl->power_real_lpf_valid = 0U;
    }
}

static void OmniUpdatePowerFeedbackScale(OmniVelocityCtrl_s *ctrl,
                                         const OmniVelocityCtrlInput_s *input,
                                         float power_est_full_w)
{
    const float limit_w = input->temp_power_limit_w;
    const float deadband_w =
        (ctrl->config.power_feedback_deadband_w > 0.0f) ?
        ctrl->config.power_feedback_deadband_w : 0.0f;
    const float min_scale =
        OmniClamp(ctrl->config.power_feedback_scale_min, 0.01f, 1.0f);
    const float down_alpha =
        OmniClamp(ctrl->config.power_feedback_down_alpha, 0.0f, 1.0f);
    const float up_alpha =
        OmniClamp(ctrl->config.power_feedback_up_alpha, 0.0f, 1.0f);
    const float local_min_ratio =
        OmniClamp(ctrl->config.power_feedback_local_min_ratio, 0.0f, 1.0f);
    float scale = ctrl->power_feedback_scale;

    if (isfinite(scale) == 0 || scale < min_scale || scale > 1.0f) {
        scale = 1.0f;
    }

    if (isfinite(limit_w) == 0 || limit_w <= 0.0f) {
        scale += up_alpha * (1.0f - scale);
        ctrl->power_feedback_scale = OmniClamp(scale, min_scale, 1.0f);
        ctrl->power_effective_limit_w = 0.0f;
        return;
    }

    if (ctrl->power_real_lpf_valid != 0U &&
        power_est_full_w > limit_w * local_min_ratio &&
        ctrl->power_real_lpf_w > limit_w + deadband_w &&
        ctrl->power_real_lpf_w > 1.0e-6f) {
        const float target_scale =
            OmniClamp(limit_w / ctrl->power_real_lpf_w, min_scale, 1.0f);
        const float alpha = (target_scale < scale) ? down_alpha : up_alpha;

        scale += alpha * (target_scale - scale);
    } else {
        scale += up_alpha * (1.0f - scale);
    }

    ctrl->power_feedback_scale = OmniClamp(scale, min_scale, 1.0f);
    ctrl->power_effective_limit_w = limit_w * ctrl->power_feedback_scale;
}

static void OmniApplyPowerVelocityScale(OmniVelocityCtrl_s *ctrl,
                                        const OmniVelocityCtrlInput_s *input,
                                        float dt)
{
    float limit_w;
    float budget_w;
    float wheel_speed_limit;
    float base_ref[OMNI_AXIS_NUM];
    float power_full;
    float sv = 1.0f;
    float sw = 1.0f;
    const uint8_t rotate_priority =
        (input->rotate_power_priority_enable != 0U) ? 1U : 0U;
    const uint8_t search_only =
        (input->power_search_only_enable != 0U) ? 1U : 0U;

    ctrl->power_vxy_scale = 1.0f;
    ctrl->power_wz_scale = 1.0f;
    ctrl->power_est_full_w = 0.0f;
    ctrl->power_est_limited_w = 0.0f;
    ctrl->power_limit_active = 0U;
    ctrl->rotate_strategy_active = 0U;
    ctrl->reachable_vxy_search_success = 0U;
    ctrl->reachable_wz_search_success = 0U;

    if (isfinite(input->temp_power_limit_w) == 0 || input->temp_power_limit_w <= 0.0f) {
        OmniUpdatePowerFeedbackScale(ctrl, input, 0.0f);
        return;
    }

    for (uint8_t axis = 0; axis < OMNI_AXIS_NUM; ++axis) {
        base_ref[axis] = ctrl->nu_ref[axis];
    }
    if (rotate_priority != 0U) {
        OmniApplyRotateWzSearchLimit(input, base_ref);
    }

    power_full = OmniEstimatePowerForScale(ctrl, input, base_ref, 1.0f, 1.0f, dt);
    ctrl->power_est_full_w = power_full;
    OmniUpdatePowerFeedbackScale(ctrl, input, power_full);

    limit_w = ctrl->power_effective_limit_w;
    budget_w = limit_w * OmniClamp(ctrl->config.power_budget_margin, 0.1f, 1.0f);
    wheel_speed_limit = OmniPowerWheelSpeedLimit(ctrl);
    ctrl->power_limit_active = 0U;

    if (isfinite(limit_w) == 0 || limit_w <= 0.0f || budget_w <= 0.0f) {
        return;
    }

    const float floor_ratio = OmniClamp(ctrl->config.power_vxy_floor_ratio, 0.0f, 1.0f);
    float vxy_abs = 0.0f;
    float dir_x = 0.0f;
    float dir_y = 0.0f;
    const uint8_t vxy_active = OmniBuildVxyDir(base_ref, &vxy_abs, &dir_x, &dir_y);

    if (rotate_priority == 0U) {
        if (search_only != 0U && vxy_active != 0U) {
            const float sv_high =
                OmniFindPureVxyScaleHigh(ctrl, base_ref, wheel_speed_limit);
            const float sv_pure_max =
                OmniFindMaxSvForPower(ctrl,
                                      input,
                                      base_ref,
                                      0.0f,
                                      0.0f,
                                      sv_high,
                                      budget_w,
                                      wheel_speed_limit,
                                      dt);

            OmniStoreReachableVxy(ctrl,
                                  vxy_abs * sv_pure_max,
                                  dir_x,
                                  dir_y,
                                  budget_w,
                                  wheel_speed_limit);
            ctrl->reachable_vxy_search_success = ctrl->reachable_vxy_valid;
        }
        ctrl->power_est_limited_w = power_full;
        ctrl->power_limit_active = 0U;
        return;
    }

    if (rotate_priority != 0U) {
        if (vxy_active == 0U) {
            const float wz_abs = OmniAbs(base_ref[OMNI_AXIS_WZ]);

            sv = 0.0f;
            sw = OmniFindMaxSwForPower(ctrl,
                                       input,
                                       base_ref,
                                       0.0f,
                                       1.0f,
                                       budget_w,
                                       wheel_speed_limit,
                                       dt);
            OmniStoreReachableWz(ctrl,
                                 wz_abs * sw,
                                 budget_w,
                                 wheel_speed_limit);
            ctrl->reachable_wz_search_success = ctrl->reachable_wz_valid;
            ctrl->power_limit_active = 1U;
            OmniApplyReferenceScale(ctrl, base_ref, sv, sw, dt);
        } else {
            const uint8_t vxy_cache_ok =
                OmniReachableVxyMatches(ctrl, dir_x, dir_y, budget_w, wheel_speed_limit);
            const uint8_t wz_cache_ok =
                OmniReachableWzMatches(ctrl, budget_w, wheel_speed_limit);

            if (vxy_cache_ok == 0U || wz_cache_ok == 0U) {
                ctrl->power_est_limited_w = power_full;
                ctrl->power_limit_active = 0U;
                return;
            }

            const float vxy_reserved =
                OmniClamp(floor_ratio * ctrl->reachable_vxy_max, 0.0f, vxy_abs);
            const float wz_abs = OmniAbs(base_ref[OMNI_AXIS_WZ]);
            float sw_high = 1.0f;

            sv = (vxy_abs > OMNI_ROTATE_VXY_ACTIVE_EPS) ?
                OmniClamp(vxy_reserved / vxy_abs, 0.0f, 1.0f) : 0.0f;
            if (wz_abs > OMNI_ROTATE_VXY_ACTIVE_EPS &&
                ctrl->reachable_wz_max > 0.0f &&
                isfinite(ctrl->reachable_wz_max) != 0) {
                sw_high = OmniClamp(ctrl->reachable_wz_max / wz_abs, 0.0f, 1.0f);
            }
            sw = OmniFindMaxSwForPower(ctrl,
                                       input,
                                       base_ref,
                                       sv,
                                       sw_high,
                                       budget_w,
                                       wheel_speed_limit,
                                       dt);
            ctrl->rotate_strategy_active = 1U;
            ctrl->power_limit_active = 1U;
            OmniApplyReferenceScale(ctrl, base_ref, sv, sw, dt);
        }

        ctrl->power_est_limited_w =
            OmniEstimatePowerForScale(ctrl,
                                      input,
                                      base_ref,
                                      ctrl->power_vxy_scale,
                                      ctrl->power_wz_scale,
                                      dt);
        return;
    }
}

static void OmniUpdateSlipRecord(OmniVelocityCtrl_s *ctrl,
                                 const OmniVelocityCtrlInput_s *input)
{
    const float threshold = (ctrl->config.slip_residual_threshold > 1.0f) ?
        ctrl->config.slip_residual_threshold : 1.0f;
    const float alpha = OmniClamp(ctrl->config.slip_lpf_alpha, 0.0f, 1.0f);
    uint8_t allow_fast_recovery = 1U;

    if (OmniAbs(input->vx_ref) > OMNI_SLIP_RECOVERY_CMD_EPS ||
        OmniAbs(input->vy_ref) > OMNI_SLIP_RECOVERY_CMD_EPS ||
        OmniAbs(input->wz_ref) > OMNI_SLIP_RECOVERY_CMD_EPS) {
        allow_fast_recovery = 0U;
    }
    for (uint8_t wheel = 0; wheel < OMNI_WHEEL_NUM; ++wheel) {
        if (OmniAbs(ctrl->force_prev[wheel]) > OMNI_SLIP_RECOVERY_FORCE_EPS_N ||
            OmniAbs(ctrl->current_ref_fit_prev[wheel]) > OMNI_SLIP_RECOVERY_CURRENT_EPS_CMD) {
            allow_fast_recovery = 0U;
            break;
        }
    }

    for (uint8_t wheel = 0; wheel < OMNI_WHEEL_NUM; ++wheel) {
        float predicted = 0.0f;
        const float measured = ctrl->config.wheel_dir[wheel] * input->wheel_speed[wheel];

        for (uint8_t axis = 0; axis < OMNI_AXIS_NUM; ++axis) {
            predicted += ctrl->h[wheel][axis] * ctrl->nu_hat[axis];
        }

        const float score = OmniClamp(OmniAbs(measured - predicted) / threshold, 0.0f, 1.0f);
        float update_alpha = alpha;
        if (allow_fast_recovery != 0U && score < ctrl->slip_prob[wheel]) {
            update_alpha = OMNI_SLIP_RECOVERY_ALPHA;
        }
        ctrl->slip_prob[wheel] += update_alpha * (score - ctrl->slip_prob[wheel]);
    }
}

static void OmniBuildCurrentRefs(OmniVelocityCtrl_s *ctrl,
                                 const OmniVelocityCtrlInput_s *input,
                                 float force_cmd[OMNI_WHEEL_NUM],
                                 uint8_t *current_limit_active,
                                 uint8_t *current_rate_limit_active,
                                 OmniVelocityCtrlOutput_s *output)
{
    const float dt = OmniSanitizeDt(ctrl, input->dt);
    const float rate_limit = input->current_rate_limit;

    *current_limit_active = 0U;
    *current_rate_limit_active = 0U;

    for (uint8_t wheel = 0; wheel < OMNI_WHEEL_NUM; ++wheel) {
        float current_limit = input->current_limit;
        float current_fit = OmniForceToCurrentFit(ctrl,
                                                  wheel,
                                                  force_cmd[wheel],
                                                  input->current_offset_enable);
        float current_limited;

        if (current_limit <= 0.0f) {
            current_limit = ctrl->config.wheel_current_limit[wheel];
        }
        if (current_limit > 0.0f) {
            current_limited = OmniClampAbs(current_fit, current_limit);
            if (current_limited != current_fit) {
                *current_limit_active = 1U;
            }
            current_fit = current_limited;
        }

        if (rate_limit > 0.0f) {
            const float max_delta = OmniAbs(rate_limit) * dt;
            current_limited = OmniClamp(current_fit,
                                        ctrl->current_ref_fit_prev[wheel] - max_delta,
                                        ctrl->current_ref_fit_prev[wheel] + max_delta);
            if (current_limited != current_fit) {
                *current_rate_limit_active = 1U;
            }
            current_fit = current_limited;
        }

        ctrl->current_ref_fit_prev[wheel] = current_fit;
        force_cmd[wheel] = OmniCurrentFitToForce(ctrl,
                                                 wheel,
                                                 current_fit,
                                                 input->current_offset_enable);
        output->wheel_current_ref[wheel] = ctrl->config.wheel_dir[wheel] * current_fit;
    }
}

static void OmniFillOutput(OmniVelocityCtrl_s *ctrl,
                           const float force_cmd[OMNI_WHEEL_NUM],
                           uint8_t limit_active,
                           uint8_t current_limit_active,
                           uint8_t current_rate_limit_active,
                           OmniVelocityCtrlOutput_s *output)
{
    for (uint8_t axis = 0; axis < OMNI_AXIS_NUM; ++axis) {
        ctrl->wrench_act_prev[axis] = 0.0f;
    }

    for (uint8_t wheel = 0; wheel < OMNI_WHEEL_NUM; ++wheel) {
        float speed_ref = 0.0f;

        for (uint8_t axis = 0; axis < OMNI_AXIS_NUM; ++axis) {
            speed_ref += ctrl->h[wheel][axis] * ctrl->nu_ref[axis];
            ctrl->wrench_act_prev[axis] += ctrl->force_h[wheel][axis] * force_cmd[wheel];
        }

        ctrl->force_prev[wheel] = force_cmd[wheel];
        output->force_cmd[wheel] = force_cmd[wheel];
        output->wheel_speed_ref[wheel] = ctrl->config.wheel_dir[wheel] * speed_ref;
    }

    for (uint8_t axis = 0; axis < OMNI_AXIS_NUM; ++axis) {
        output->nu_ref[axis] = ctrl->nu_ref[axis];
        output->nu_hat[axis] = ctrl->nu_hat[axis];
        output->lqi_ref[axis] = ctrl->lqi_ref[axis];
        output->lqi_feedback[axis] = ctrl->lqi_feedback[axis];
        output->lqi_error[axis] = ctrl->lqi_error[axis];
        output->lqi_integral[axis] = ctrl->integral[axis];
        output->nu_eso[axis] = ctrl->nu_eso[axis];
        output->dist_hat[axis] = ctrl->dist_hat[axis];
        output->wrench_des[axis] = ctrl->wrench_des_prev[axis];
        output->wrench_act[axis] = ctrl->wrench_act_prev[axis];
    }
    for (uint8_t wheel = 0; wheel < OMNI_WHEEL_NUM; ++wheel) {
        output->slip_prob[wheel] = ctrl->slip_prob[wheel];
    }
    output->limit_active = (uint8_t)(limit_active |
                                     current_limit_active |
                                     current_rate_limit_active |
                                     ctrl->power_limit_active);
    output->current_limit_active = current_limit_active;
    output->current_rate_limit_active = current_rate_limit_active;
    output->lqi_wz_feedback_source = ctrl->lqi_wz_feedback_source;
    output->power_limit_active = ctrl->power_limit_active;
    output->power_real_valid = ctrl->power_real_valid;
    output->power_vxy_scale = ctrl->power_vxy_scale;
    output->power_wz_scale = ctrl->power_wz_scale;
    output->power_est_full_w = ctrl->power_est_full_w;
    output->power_est_limited_w = ctrl->power_est_limited_w;
    output->power_real_w = ctrl->power_real_w;
    output->power_feedback_scale = ctrl->power_feedback_scale;
    output->power_effective_limit_w = ctrl->power_effective_limit_w;
    output->reachable_vxy_valid = ctrl->reachable_vxy_valid;
    output->reachable_wz_valid = ctrl->reachable_wz_valid;
    output->rotate_strategy_active = ctrl->rotate_strategy_active;
    output->reachable_vxy_search_success = ctrl->reachable_vxy_search_success;
    output->reachable_wz_search_success = ctrl->reachable_wz_search_success;
    output->reachable_vxy_max = ctrl->reachable_vxy_max;
    output->reachable_wz_max = ctrl->reachable_wz_max;
    output->reachable_vxy_dir_x = ctrl->reachable_vxy_dir_x;
    output->reachable_vxy_dir_y = ctrl->reachable_vxy_dir_y;
}

void OmniVelocityCtrl_GetDefaultConfig(OmniVelocityCtrlConfig_s *config)
{
    if (config == 0) {
        return;
    }

    memset(config, 0, sizeof(*config));
    config->wheel_base_mm = WHEEL_BASE;
    config->track_width_mm = TRACK_WIDTH;
    config->control_dt_s = 0.001f;
    /* 实车左侧前后轮速度方向与矩阵定义相反，这里统一翻转速度参考和反馈反解符号。 */
    config->wheel_dir[OMNI_WHEEL_LF] = -1.0f;
    config->wheel_dir[OMNI_WHEEL_RF] = 1.0f;
    config->wheel_dir[OMNI_WHEEL_RB] = 1.0f;
    config->wheel_dir[OMNI_WHEEL_LB] = -1.0f;
    config->speed_acc_limit = 300000.0f;
    config->wz_acc_limit = 60000.0f;
    config->kp[OMNI_AXIS_VX] = 8.0f;
    config->kp[OMNI_AXIS_VY] = 8.0f;
    config->kp[OMNI_AXIS_WZ] = 6.0f;
    config->ki[OMNI_AXIS_VX] = 0.0f;
    config->ki[OMNI_AXIS_VY] = 0.0f;
    config->ki[OMNI_AXIS_WZ] = 0.0f;
    config->integral_limit[OMNI_AXIS_VX] = 1.5f;
    config->integral_limit[OMNI_AXIS_VY] = 1.5f;
    config->integral_limit[OMNI_AXIS_WZ] = 3.0f;
    config->mass[OMNI_AXIS_VX] = 16.9f;
    config->mass[OMNI_AXIS_VY] = 16.9f;
    config->mass[OMNI_AXIS_WZ] = 0.50f;
    config->damping[OMNI_AXIS_VX] = 0.0f;
    config->damping[OMNI_AXIS_VY] = 0.0f;
    config->damping[OMNI_AXIS_WZ] = 0.0f;
    for (uint8_t wheel = 0; wheel < OMNI_WHEEL_NUM; ++wheel) {
        config->force_to_cmd_pos[wheel] = 288.0f;
        config->force_to_cmd_neg[wheel] = 288.0f;
        config->current_offset[wheel] = 0.0f;
        config->wheel_current_limit[wheel] = 3000.0f;
    }
    config->force_to_cmd_pos[OMNI_WHEEL_LF] = 192.7f;
    config->force_to_cmd_neg[OMNI_WHEEL_LF] = 150.7f;
    config->force_to_cmd_pos[OMNI_WHEEL_RF] = 263.8f;
    config->force_to_cmd_neg[OMNI_WHEEL_RF] = 143.9f;
    config->force_to_cmd_pos[OMNI_WHEEL_RB] = 801.5f;
    config->force_to_cmd_neg[OMNI_WHEEL_RB] = 486.2f;
    config->force_to_cmd_pos[OMNI_WHEEL_LB] = 848.1f;
    config->force_to_cmd_neg[OMNI_WHEEL_LB] = 285.9f;
    config->eso_enable = 1U;
    config->eso_compensate_enable = 1U;
    config->eso_compensate_gain[OMNI_AXIS_VX] = 1.0f;
    config->eso_compensate_gain[OMNI_AXIS_VY] = 1.0f;
    config->eso_compensate_gain[OMNI_AXIS_WZ] = 0.0f;
    config->eso_bandwidth[OMNI_AXIS_VX] = 60.0f;
    config->eso_bandwidth[OMNI_AXIS_VY] = 60.0f;
    config->eso_bandwidth[OMNI_AXIS_WZ] = 45.0f;
    config->eso_comp_static_deadband_v = 0.03f;
    config->eso_comp_static_deadband_wz = 0.05f;
    config->allocator_regularization = 1.0e-3f;
    config->slip_residual_threshold = 800.0f;
    config->slip_lpf_alpha = 0.05f;
    config->slip_force_enable = 1U;
    config->slip_force_gain = 10.0f;
    config->slip_force_min_weight = 1.0f;//0.7f;
    config->power_vxy_floor_ratio = 0.6f;
    config->power_budget_margin = OMNI_POWER_BUDGET_MARGIN_DEFAULT;
    config->power_wheel_speed_limit = OMNI_POWER_WHEEL_SPEED_LIMIT_DEFAULT;
    config->power_feedback_deadband_w = OMNI_POWER_FEEDBACK_DEFAULT_DEADBAND_W;
    config->power_feedback_scale_min = OMNI_POWER_FEEDBACK_DEFAULT_SCALE_MIN;
    config->power_feedback_down_alpha = OMNI_POWER_FEEDBACK_DEFAULT_DOWN_ALPHA;
    config->power_feedback_up_alpha = OMNI_POWER_FEEDBACK_DEFAULT_UP_ALPHA;
    config->power_feedback_lpf_tau_s = OMNI_POWER_FEEDBACK_DEFAULT_LPF_TAU_S;
    config->power_feedback_local_min_ratio = OMNI_POWER_FEEDBACK_DEFAULT_LOCAL_MIN_RATIO;
}

void OmniVelocityCtrl_Init(OmniVelocityCtrl_s *ctrl, const OmniVelocityCtrlConfig_s *config)
{
    OmniVelocityCtrlConfig_s default_config;

    if (ctrl == 0) {
        return;
    }

    memset(ctrl, 0, sizeof(*ctrl));
    if (config == 0) {
        OmniVelocityCtrl_GetDefaultConfig(&default_config);
        ctrl->config = default_config;
    } else {
        ctrl->config = *config;
    }

    if (ctrl->config.control_dt_s <= 0.0f) {
        ctrl->config.control_dt_s = 0.001f;
    }
    if (ctrl->config.allocator_regularization <= 0.0f) {
        ctrl->config.allocator_regularization = 1.0e-3f;
    }
    if (ctrl->config.eso_comp_static_deadband_v < 0.0f) {
        ctrl->config.eso_comp_static_deadband_v = 0.0f;
    }
    if (ctrl->config.eso_comp_static_deadband_wz < 0.0f) {
        ctrl->config.eso_comp_static_deadband_wz = 0.0f;
    }
    if (ctrl->config.slip_force_gain < 0.0f) {
        ctrl->config.slip_force_gain = 0.0f;
    }
    if (ctrl->config.slip_force_min_weight < 0.0f) {
        ctrl->config.slip_force_min_weight = 0.0f;
    } else if (ctrl->config.slip_force_min_weight > 1.0f) {
        ctrl->config.slip_force_min_weight = 1.0f;
    }
    if (ctrl->config.power_vxy_floor_ratio < 0.0f) {
        ctrl->config.power_vxy_floor_ratio = 0.0f;
    } else if (ctrl->config.power_vxy_floor_ratio > 1.0f) {
        ctrl->config.power_vxy_floor_ratio = 1.0f;
    }
    if (ctrl->config.power_budget_margin <= 0.0f ||
        ctrl->config.power_budget_margin > 1.0f) {
        ctrl->config.power_budget_margin = OMNI_POWER_BUDGET_MARGIN_DEFAULT;
    }
    if (ctrl->config.power_wheel_speed_limit <= 0.0f ||
        isfinite(ctrl->config.power_wheel_speed_limit) == 0) {
        ctrl->config.power_wheel_speed_limit = OMNI_POWER_WHEEL_SPEED_LIMIT_DEFAULT;
    }
    if (ctrl->config.power_feedback_deadband_w < 0.0f) {
        ctrl->config.power_feedback_deadband_w = 0.0f;
    }
    if (ctrl->config.power_feedback_scale_min <= 0.0f ||
        ctrl->config.power_feedback_scale_min > 1.0f) {
        ctrl->config.power_feedback_scale_min = OMNI_POWER_FEEDBACK_DEFAULT_SCALE_MIN;
    }
    if (ctrl->config.power_feedback_down_alpha < 0.0f ||
        ctrl->config.power_feedback_down_alpha > 1.0f) {
        ctrl->config.power_feedback_down_alpha = OMNI_POWER_FEEDBACK_DEFAULT_DOWN_ALPHA;
    }
    if (ctrl->config.power_feedback_up_alpha < 0.0f ||
        ctrl->config.power_feedback_up_alpha > 1.0f) {
        ctrl->config.power_feedback_up_alpha = OMNI_POWER_FEEDBACK_DEFAULT_UP_ALPHA;
    }
    if (ctrl->config.power_feedback_lpf_tau_s <= 0.0f ||
        isfinite(ctrl->config.power_feedback_lpf_tau_s) == 0) {
        ctrl->config.power_feedback_lpf_tau_s = OMNI_POWER_FEEDBACK_DEFAULT_LPF_TAU_S;
    }
    if (ctrl->config.power_feedback_local_min_ratio <= 0.0f ||
        ctrl->config.power_feedback_local_min_ratio > 1.0f ||
        isfinite(ctrl->config.power_feedback_local_min_ratio) == 0) {
        ctrl->config.power_feedback_local_min_ratio =
            OMNI_POWER_FEEDBACK_DEFAULT_LOCAL_MIN_RATIO;
    }
    for (uint8_t wheel = 0; wheel < OMNI_WHEEL_NUM; ++wheel) {
        if (ctrl->config.wheel_dir[wheel] == 0.0f) {
            ctrl->config.wheel_dir[wheel] = 1.0f;
        }
        if (ctrl->config.force_to_cmd_pos[wheel] <= 1.0e-6f) {
            ctrl->config.force_to_cmd_pos[wheel] = 200.0f;
        }
        if (ctrl->config.force_to_cmd_neg[wheel] <= 1.0e-6f) {
            ctrl->config.force_to_cmd_neg[wheel] = ctrl->config.force_to_cmd_pos[wheel];
        }
        if (ctrl->config.wheel_current_limit[wheel] <= 0.0f) {
            ctrl->config.wheel_current_limit[wheel] = 3000.0f;
        }
    }

    OmniBuildMatrix(ctrl);
    ctrl->power_feedback_scale = 1.0f;
    ctrl->inited = 1U;
}

void OmniVelocityCtrl_Reset(OmniVelocityCtrl_s *ctrl)
{
    OmniVelocityCtrlConfig_s saved_config;

    if (ctrl == 0) {
        return;
    }

    saved_config = ctrl->config;
    memset(ctrl, 0, sizeof(*ctrl));
    ctrl->config = saved_config;
    OmniBuildMatrix(ctrl);
    ctrl->power_feedback_scale = 1.0f;
    ctrl->inited = 1U;
}

void OmniVelocityCtrl_ClearReachableCache(OmniVelocityCtrl_s *ctrl)
{
    if (ctrl == 0) {
        return;
    }

    OmniClearReachableCacheState(ctrl);
}

void OmniVelocityCtrl_ExportReachableCache(const OmniVelocityCtrl_s *ctrl,
                                           OmniReachableCache_s *cache)
{
    if (ctrl == 0 || cache == 0) {
        return;
    }

    memset(cache, 0, sizeof(*cache));
    cache->reachable_vxy_valid = ctrl->reachable_vxy_valid;
    cache->reachable_wz_valid = ctrl->reachable_wz_valid;
    cache->reachable_vxy_max = ctrl->reachable_vxy_max;
    cache->reachable_wz_max = ctrl->reachable_wz_max;
    cache->reachable_vxy_dir_x = ctrl->reachable_vxy_dir_x;
    cache->reachable_vxy_dir_y = ctrl->reachable_vxy_dir_y;
    cache->reachable_vxy_budget_w = ctrl->reachable_vxy_budget_w;
    cache->reachable_wz_budget_w = ctrl->reachable_wz_budget_w;
    cache->reachable_vxy_wheel_speed_limit = ctrl->reachable_vxy_wheel_speed_limit;
    cache->reachable_wz_wheel_speed_limit = ctrl->reachable_wz_wheel_speed_limit;
}

void OmniVelocityCtrl_ImportReachableCache(OmniVelocityCtrl_s *ctrl,
                                           const OmniReachableCache_s *cache)
{
    if (ctrl == 0 || cache == 0) {
        return;
    }

    OmniClearReachableCacheState(ctrl);
    if (cache->reachable_vxy_valid != 0U) {
        OmniStoreReachableVxy(ctrl,
                              cache->reachable_vxy_max,
                              cache->reachable_vxy_dir_x,
                              cache->reachable_vxy_dir_y,
                              cache->reachable_vxy_budget_w,
                              cache->reachable_vxy_wheel_speed_limit);
    }
    if (cache->reachable_wz_valid != 0U) {
        OmniStoreReachableWz(ctrl,
                             cache->reachable_wz_max,
                             cache->reachable_wz_budget_w,
                             cache->reachable_wz_wheel_speed_limit);
    }
    ctrl->reachable_vxy_search_success = 0U;
    ctrl->reachable_wz_search_success = 0U;
    ctrl->rotate_strategy_active = 0U;
}

/*
 * omni 速度控制主链路：
 * 1. 对 vx/vy/wz 原始参考做加速度限幅。
 * 2. 用四轮编码器速度反解车体速度；wz 闭环优先使用输入的 IMU yaw gyro。
 * 3. ESO 更新速度/扰动估计，并在功率预算下分配可达 vxy/wz 参考。
 * 4. LQI 根据降额后的参考、反馈和扰动补偿生成期望合力/力矩。
 * 5. 更新滑移概率并在多轮滑移时降额，再把期望合力/力矩分配成四轮轮力。
 * 6. 应用后置功率硬限幅、电流幅值限制和电流变化率限制，最终转成 DJI raw current cmd。
 */
void OmniVelocityCtrl_Step(OmniVelocityCtrl_s *ctrl,
                           const OmniVelocityCtrlInput_s *input,
                           OmniVelocityCtrlOutput_s *output)
{
    float force_cmd[OMNI_WHEEL_NUM] = {0.0f};
    float dt;
    uint8_t limit_active = 0U;
    uint8_t current_limit_active = 0U;
    uint8_t current_rate_limit_active = 0U;

    if (ctrl == 0 || input == 0 || output == 0) {
        return;
    }

    if (ctrl->inited == 0U) {
        OmniVelocityCtrl_Init(ctrl, 0);
    }

    memset(output, 0, sizeof(*output));
    dt = OmniSanitizeDt(ctrl, input->dt);

    OmniLimitReference(ctrl, input, dt);
    OmniEstimateVelocity(ctrl, input);
    OmniUpdateLqiFeedback(ctrl, input);
    OmniUpdateEso(ctrl, dt);
    OmniUpdatePowerFeedback(ctrl, input, dt);
    OmniApplyPowerVelocityScale(ctrl, input, dt);
    OmniUpdateLqiReference(ctrl);
    OmniCalcDesiredWrench(ctrl, dt);
    OmniApplyExternalWrenchFeedforward(ctrl, input);
    OmniUpdateSlipRecord(ctrl, input);
    OmniApplyMultiSlipWrenchDerate(ctrl);
    OmniAllocateForce(ctrl, input, force_cmd, &limit_active);
    OmniBuildCurrentRefs(ctrl,
                         input,
                         force_cmd,
                         &current_limit_active,
                         &current_rate_limit_active,
                         output);
    ctrl->power_est_limited_w =
        OmniEstimatePositiveWheelPower(ctrl, input, force_cmd, ctrl->nu_ref);
    OmniFillOutput(ctrl,
                   force_cmd,
                   limit_active,
                   current_limit_active,
                   current_rate_limit_active,
                   output);
}
