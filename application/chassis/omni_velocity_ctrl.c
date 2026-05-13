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
#define OMNI_MULTI_SLIP_MIN_WRENCH_SCALE 0.30f
#define OMNI_MULTI_SLIP_MAX_TRIGGER 0.90f
#define OMNI_MULTI_SLIP_AVG_TRIGGER 0.45f
#define OMNI_SLIP_RECOVERY_ALPHA 0.20f
#define OMNI_SLIP_RECOVERY_CMD_EPS 1.0f
#define OMNI_SLIP_RECOVERY_FORCE_EPS_N 2.0f
#define OMNI_SLIP_RECOVERY_CURRENT_EPS_CMD 800.0f

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
     * 根据 chassis_reH.csv 遥控数据直接命令单位拟合得到的普通 X 型 omni 轮端速度矩阵。
     * 输入坐标为 vx 前向、vy 左向、wz 逆时针为正；轮序固定为 LF/RF/RB/LB。
     * DJI 电机反馈和参考仍是电机轴单位，底盘层用 268/17 减速比换算到轮端单位。
     * wheel_dir 只修正电机安装方向，不负责减速比换算。
     */
    ctrl->h[OMNI_WHEEL_LF][OMNI_AXIS_VX] = 0.0347094147f;
    ctrl->h[OMNI_WHEEL_LF][OMNI_AXIS_VY] = -0.0361355412f;
    ctrl->h[OMNI_WHEEL_LF][OMNI_AXIS_WZ] = -0.267720510f;

    ctrl->h[OMNI_WHEEL_RF][OMNI_AXIS_VX] = 0.0358944042f;
    ctrl->h[OMNI_WHEEL_RF][OMNI_AXIS_VY] = 0.0368013157f;
    ctrl->h[OMNI_WHEEL_RF][OMNI_AXIS_WZ] = 0.266192090f;

    ctrl->h[OMNI_WHEEL_RB][OMNI_AXIS_VX] = 0.0351562327f;
    ctrl->h[OMNI_WHEEL_RB][OMNI_AXIS_VY] = -0.0351616775f;
    ctrl->h[OMNI_WHEEL_RB][OMNI_AXIS_WZ] = 0.262277860f;

    ctrl->h[OMNI_WHEEL_LB][OMNI_AXIS_VX] = 0.0358132523f;
    ctrl->h[OMNI_WHEEL_LB][OMNI_AXIS_VY] = 0.0357837394f;
    ctrl->h[OMNI_WHEEL_LB][OMNI_AXIS_WZ] = -0.264667993f;

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

static void OmniCalcDesiredWrench(OmniVelocityCtrl_s *ctrl, float dt)
{
    for (uint8_t i = 0; i < OMNI_AXIS_NUM; ++i) {
        const float err = ctrl->lqi_ref[i] - ctrl->lqi_feedback[i];
        const float mass = (OmniAbs(ctrl->config.mass[i]) > 1.0e-6f) ?
            ctrl->config.mass[i] : 1.0f;

        ctrl->lqi_error[i] = err;
        if (OmniAbs(ctrl->config.ki[i]) > 1.0e-6f) {
            ctrl->integral[i] += err * dt;
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

static void OmniSolveFreeForces(OmniVelocityCtrl_s *ctrl,
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
        return 0.0f;
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
    for (uint8_t wheel = 0; wheel < OMNI_WHEEL_NUM; ++wheel) {
        float current_limit = input->current_limit;
        float pos_k = ctrl->config.force_to_cmd_pos[wheel];
        float neg_k = ctrl->config.force_to_cmd_neg[wheel];
        float usable_current;

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

        usable_current = OmniAbs(current_limit);
        if (input->current_offset_enable != 0U) {
            usable_current -= OmniAbs(ctrl->config.current_offset[wheel]);
            if (usable_current < 0.0f) {
                usable_current = 0.0f;
            }
        }

        force_max[wheel] = usable_current / pos_k;
        force_min[wheel] = -usable_current / neg_k;
    }
}

static void OmniAllocateForce(OmniVelocityCtrl_s *ctrl,
                              const OmniVelocityCtrlInput_s *input,
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
            wrench_rem[axis] = ctrl->wrench_des_prev[axis];
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

static void OmniApplyTemporaryPowerLimit(const OmniVelocityCtrl_s *ctrl,
                                         const OmniVelocityCtrlInput_s *input,
                                         float force_cmd[OMNI_WHEEL_NUM],
                                         uint8_t *limit_active)
{
    const float limit_w = input->temp_power_limit_w;
    const float wheel_radius_m = OmniWheelRadiusM();
    float estimated_power_w = 0.0f;

    if (limit_w <= 0.0f) {
        return;
    }

    for (uint8_t wheel = 0; wheel < OMNI_WHEEL_NUM; ++wheel) {
        const float wheel_speed_fit =
            ctrl->config.wheel_dir[wheel] * input->wheel_speed[wheel];
        const float wheel_speed_mps = wheel_speed_fit * DEGREE_2_RAD * wheel_radius_m;
        const float wheel_power_w = force_cmd[wheel] * wheel_speed_mps;

        if (wheel_power_w > 0.0f) {
            estimated_power_w += wheel_power_w;
        }
    }

    if (estimated_power_w > limit_w && estimated_power_w > 1.0e-6f) {
        const float scale = limit_w / estimated_power_w;

        for (uint8_t wheel = 0; wheel < OMNI_WHEEL_NUM; ++wheel) {
            force_cmd[wheel] *= scale;
        }
        if (limit_active != 0) {
            *limit_active = 1U;
        }
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
                                     current_rate_limit_active);
    output->current_limit_active = current_limit_active;
    output->current_rate_limit_active = current_rate_limit_active;
    output->lqi_wz_feedback_source = ctrl->lqi_wz_feedback_source;
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
        config->current_offset[wheel] = 0;//1300.0f;
        config->wheel_current_limit[wheel] = 3000.0f;
    }
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
    config->slip_force_min_weight = 0.0f;
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
    ctrl->inited = 1U;
}

/*
 * omni 速度控制主链路：
 * 1. 对 vx/vy/wz 参考做加速度限幅，并换算成 LQI 使用的 SI 单位参考。
 * 2. 用四轮编码器速度反解车体速度；wz 闭环优先使用输入的 IMU yaw gyro。
 * 3. ESO 更新速度/扰动估计，LQI 根据参考、反馈和扰动补偿生成期望合力/力矩。
 * 4. 更新滑移概率并在多轮滑移时降额，再把期望合力/力矩分配成四轮轮力。
 * 5. 应用临时功率限制、电流幅值限制和电流变化率限制，最终转成 DJI raw current cmd。
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
    OmniUpdateLqiReference(ctrl);
    OmniEstimateVelocity(ctrl, input);
    OmniUpdateLqiFeedback(ctrl, input);
    OmniUpdateEso(ctrl, dt);
    OmniCalcDesiredWrench(ctrl, dt);
    OmniUpdateSlipRecord(ctrl, input);
    OmniApplyMultiSlipWrenchDerate(ctrl);
    OmniAllocateForce(ctrl, input, force_cmd, &limit_active);
    OmniApplyTemporaryPowerLimit(ctrl, input, force_cmd, &limit_active);
    OmniBuildCurrentRefs(ctrl,
                         input,
                         force_cmd,
                         &current_limit_active,
                         &current_rate_limit_active,
                         output);
    OmniFillOutput(ctrl,
                   force_cmd,
                   limit_active,
                   current_limit_active,
                   current_rate_limit_active,
                   output);
}
