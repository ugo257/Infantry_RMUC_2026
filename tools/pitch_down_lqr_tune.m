%% pitch_down_lqr_tune.m
% Tune pitch-down state-space LQR gains for the current firmware implementation.
%
% Firmware convention in application/gimbal/gimbal.c:
%   e_theta = measure - ref;
%   e_omega = speed;
%   tau_lqr = -k_theta * e_theta - k_omega * e_omega;
%   tau_cmd = tau_lqr + tau_eso_comp;
%   ESO beta is recomputed every PitchLQRESO_Calc() call from the active K.
%
% Pitch-down keeps its own motor position, not IMU pitch:
%   ref     = pitch_down_ref_limited_pos_rad
%   measure = pitch_motor_down->measure.pos
%   speed   = filtered pitch_motor_down->measure.vel
%
% The firmware down model is the explicit continuous state-space plant:
%   x = [e_theta; omega]
%   x_dot = A*x + Bu*u
%   A  = [0, 1;
%         0, -Bv / J]
%   Bu = [0;
%         1 / J]
%
% Gravity and hold-I feedforward are disabled in the LQR torque branch. The
% ESO compensation is responsible for residual low-frequency/nonlinear terms.

clear; clc; close all;

%% Firmware-synchronized plant/control parameters
modelPreset = 'ozone_260507_state_space';

J = 0.0411;                % kg*m^2, PITCH_DOWN_MODEL_J_KG_M2
Bv = 0.0869;               % N*m*s/rad, PITCH_DOWN_MODEL_B_NMS_RAD
Ts = 0.001;                % s, GimbalTask_PITCH nominal period
torqueLimit = 7.0;         % Nm, PITCH_DOWN_LQR_TORQUE_LIMIT_NM
torqueSlewRate = 1000.0;   % Nm/s, PITCH_DOWN_TORQUE_SLEW_RATE_NM_S
velocityLimit = 10.0;      % rad/s, PITCH_DOWN_VEL_LIMIT_RAD_S
velocityLpfRc = 0.030;     % s, PITCH_DOWN_VEL_LPF_DEFAULT_RC_S
enablePlot = false;        % true: show saturated step response

pitch_down_eso_enable = true;       % PITCH_DOWN_ESO_ENABLE
pitch_down_eso_alpha = 1.5;         % PITCH_DOWN_ESO_ALPHA
pitch_down_eso_comp_gain = 1.0;     % PITCH_DOWN_ESO_COMP_GAIN
pitch_down_eso_comp_limit_nm = 2.0; % PITCH_DOWN_ESO_COMP_LIMIT_NM
pitch_down_eso_w0_min_rad_s = 0.0;  % PITCH_DOWN_ESO_W0_MIN_RAD_S
pitch_down_eso_w0_max_rad_s = 80.0; % PITCH_DOWN_ESO_W0_MAX_RAD_S

%% Tunable LQR weights
% The printed firmware copy block uses the K derived from these Q/R weights.
q_theta = 200.0;          % PITCH_DOWN_MODEL_Q_THETA
q_omega = 0.02;             % PITCH_DOWN_MODEL_Q_OMEGA
r_tau   = 0.2;             % PITCH_DOWN_MODEL_R_TORQUE

%% Direct K tuning
% Set useManualTargetGain=true only when you explicitly want to bypass Q/R.
targetKTheta = 42.426407;
targetKOmega = 2.033242;
useManualTargetGain = false; % false: print QR model K; true: print target K

Q = diag([q_theta, q_omega]);
R = r_tau;

%% Plant
A = [0, 1;
     0, -Bv / J];

Bu = [0;
      1 / J];

if rank([Bu, A * Bu]) < size(A, 1)
    error('Pitch-down plant is not controllable. Check J and Bv.');
end

%% Continuous LQR
% Firmware uses this same closed-form continuous CARE solution for:
%   A = [0, 1; 0, -Bv/J], Bu = [0; 1/J], Q = diag(q_theta, q_omega), R = r_tau
K_model = pitch_down_analytic_care_gain(J, Bv, q_theta, q_omega, r_tau);

if exist('lqr', 'file') == 2
    [Kc, ~, poles_c] = lqr(A, Bu, Q, R);
else
    Kc = K_model;
    poles_c = eig(A - Bu * Kc);
end

if exist('ss', 'file') == 2 && exist('c2d', 'file') == 2 && exist('dlqr', 'file') == 2
    sys_c = ss(A, Bu, eye(2), zeros(2, 1));
    sys_d = c2d(sys_c, Ts);
    [Kd, ~, poles_d] = dlqr(sys_d.A, sys_d.B, Q, R);
else
    sys_d = [];
    Kd = [NaN, NaN];
    poles_d = [NaN; NaN];
end

%% Choose gain to copy into firmware
K = K_model;

if useManualTargetGain
    K = [targetKTheta, targetKOmega];
end

fprintf('K = [%.6f %.6f]\n', K(1), K(2));
if useManualTargetGain
    fprintf('K source = manual targetKTheta/targetKOmega\n');
else
    fprintf('K source = QR model, Q = diag([%.9g %.9g]), R = %.9g\n', ...
            q_theta, q_omega, r_tau);
end

eso = pitch_down_eso_preview(J, Bv, K, ...
                             pitch_down_eso_enable, ...
                             pitch_down_eso_alpha, ...
                             pitch_down_eso_w0_min_rad_s, ...
                             pitch_down_eso_w0_max_rad_s);

fprintf('\n=== Firmware copy block ===\n');
fprintf('#define PITCH_DOWN_LQR_K_THETA              %.6ff\n', K(1));
fprintf('#define PITCH_DOWN_LQR_K_OMEGA              %.6ff\n', K(2));
fprintf('#define PITCH_DOWN_TORQUE_SLEW_RATE_NM_S    %.6ff\n', torqueSlewRate);
fprintf('#define PITCH_DOWN_VEL_LIMIT_RAD_S          %.6ff\n', velocityLimit);
fprintf('#define PITCH_DOWN_ESO_ENABLE               %dU\n', pitch_down_eso_enable);
fprintf('#define PITCH_DOWN_ESO_ALPHA                %.9gf\n', pitch_down_eso_alpha);
fprintf('#define PITCH_DOWN_ESO_COMP_GAIN            %.9gf\n', pitch_down_eso_comp_gain);
fprintf('#define PITCH_DOWN_ESO_COMP_LIMIT_NM        %.9gf\n', pitch_down_eso_comp_limit_nm);
fprintf('#define PITCH_DOWN_ESO_W0_MIN_RAD_S         %.9gf\n', pitch_down_eso_w0_min_rad_s);
fprintf('#define PITCH_DOWN_ESO_W0_MAX_RAD_S         %.9gf\n', pitch_down_eso_w0_max_rad_s);

fprintf('\n=== ESO beta preview from active K ===\n');
fprintf('b0     = %.9g\n', eso.b0);
fprintf('wn     = %.9g rad/s\n', eso.wn);
fprintf('sigma  = %.9g rad/s\n', eso.sigma);
fprintf('w_cl   = %.9g rad/s\n', eso.w_cl);
fprintf('w0     = %.9g rad/s\n', eso.w0);
fprintf('beta1  = %.9g\n', eso.beta1);
fprintf('beta2  = %.9g\n', eso.beta2);
fprintf('beta3  = %.9g\n', eso.beta3);

%% Simple saturated step simulation
refStep = 0.20;  % rad
Tend = 1.5;      % s
N = round(Tend / Ts);

x = [-refStep; 0];  % measure starts at 0, ref steps to +refStep
omegaFilt = x(2);
uLast = 0.0;
log_t = (0:N-1)' * Ts;
log_x = zeros(N, 2);
log_u = zeros(N, 1);
log_omega_filt = zeros(N, 1);

for i = 1:N
    velocityAlpha = Ts / (velocityLpfRc + Ts);
    omegaFilt = omegaFilt + velocityAlpha * (x(2) - omegaFilt);

    u_raw = -K(1) * x(1) - K(2) * omegaFilt;
    u = clamp_scalar(u_raw, -torqueLimit, torqueLimit);
    maxDeltaU = torqueSlewRate * Ts;
    u = clamp_scalar(u, uLast - maxDeltaU, uLast + maxDeltaU);
    if (omegaFilt >= velocityLimit && u > 0.0) || ...
       (omegaFilt <= -velocityLimit && u < 0.0)
        u = 0.0;
    end
    uLast = u;

    xdot = A * x + Bu * u;
    x = x + Ts * xdot;

    log_x(i, :) = x.';
    log_u(i) = u;
    log_omega_filt(i) = omegaFilt;
end

if enablePlot
    figure('Name', 'Pitch-down LQR saturated step');
    subplot(2, 1, 1);
    plot(log_t, log_x(:, 1), 'LineWidth', 1.2);
    grid on;
    ylabel('e_\theta = measure-ref (rad)');

    subplot(2, 1, 2);
    plot(log_t, log_u, 'LineWidth', 1.2);
    grid on;
    ylabel('\tau (Nm)');
    xlabel('Time (s)');
end

function y = clamp_scalar(x, lower, upper)
    y = min(max(x, lower), upper);
end

function eso = pitch_down_eso_preview(J, Bv, K, enable, alpha, w0_min, w0_max)
    eso = struct('b0', 0.0, 'wn', 0.0, 'sigma', 0.0, 'w_cl', 0.0, ...
                 'w0', 0.0, 'beta1', 0.0, 'beta2', 0.0, 'beta3', 0.0);

    if ~enable || J <= 0.0
        return;
    end

    eso.b0 = 1.0 / J;
    wn_term = K(1) / J;
    if wn_term > 0.0
        eso.wn = sqrt(wn_term);
    end
    eso.sigma = max((Bv + K(2)) / (2.0 * J), 0.0);
    eso.w_cl = max(eso.wn, eso.sigma);

    alpha = max(alpha, 0.0);
    w0_min = max(w0_min, 0.0);
    w0_max = max(w0_max, 0.0);
    if w0_max > 0.0 && w0_min > w0_max
        w0_min = w0_max;
    end

    eso.w0 = alpha * eso.w_cl;
    if eso.w0 < w0_min
        eso.w0 = w0_min;
    end
    if w0_max > 0.0 && eso.w0 > w0_max
        eso.w0 = w0_max;
    end

    eso.beta1 = 3.0 * eso.w0;
    eso.beta2 = 3.0 * eso.w0 * eso.w0;
    eso.beta3 = eso.w0 * eso.w0 * eso.w0;
end

function K = pitch_down_analytic_care_gain(J, Bv, q_theta, q_omega, r_tau)
    if J <= 0.0
        error('J must be positive.');
    end
    if q_theta < 0.0 || q_omega < 0.0
        error('Q weights must be non-negative.');
    end
    if r_tau <= 0.0
        error('R must be positive.');
    end

    sqrt_qr = sqrt(q_theta * r_tau);
    k_theta = sqrt(q_theta / r_tau);
    omega_term = Bv^2 + ...
        (2.0 * J * sqrt_qr + q_omega) / r_tau;
    k_omega = sqrt(omega_term) - Bv;

    K = [k_theta, k_omega];
end
