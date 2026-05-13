%% pitch_up_lqr_tune.m
% Tune pitch-up LQR gains for the current firmware implementation.
%
% Firmware convention:
%   e_theta = measure - ref;
%   e_omega = speed;
%   tau_lqr = -k_theta * e_theta - k_omega * e_omega;
%
% Plant model used by LQR and firmware ESO preview:
%   theta_dot = omega
%   omega_dot = (motorSign * tau_lqr - Bv * omega) / J

clear; clc; close all;

%% Tunable plant parameters
J = 0.036;          % kg*m^2, equivalent pitch-up inertia
Bv = 0.032;         % N*m*s/rad, viscous damping
Ts = 0.001;         % s, GimbalTask_PITCH nominal period
motorSign = 1;      % +1: positive torque increases IMU pitch angle; use -1 if measured opposite
enablePlot = false; % true: show static step-response figure; false: print only

%% Firmware pitch-up ESO trial parameters
pitch_up_eso_trial_enable = 1;
pitch_up_eso_enable = 1;
pitch_up_eso_alpha = 1.5;
pitch_up_eso_comp_gain = 0.5;
pitch_up_eso_comp_limit_nm = 1.2;
pitch_up_eso_w0_min_rad_s = 0.0;
pitch_up_eso_w0_max_rad_s = 80.0;

%% Tunable LQR weights
q_theta = 80.0;     % angle error weight; larger is more aggressive
q_omega = 0.05;     % angular velocity weight; larger adds damping
r_tau   = 0.015;      % torque cost; larger is more conservative

Q = diag([q_theta, q_omega]);
R = r_tau;

%% Plant
A = [0, 1;
     0, -Bv / J];

B = [0;
     motorSign / J];

if rank(ctrb(A, B)) < size(A, 1)
    error('Pitch-up plant is not controllable. Check J, Bv, and motorSign.');
end

%% Continuous and discrete LQR
[Kc, ~, poles_c] = lqr(A, B, Q, R);

sys_c = ss(A, B, eye(2), zeros(2, 1));
sys_d = c2d(sys_c, Ts);
[Kd, ~, poles_d] = dlqr(sys_d.A, sys_d.B, Q, R);

%% Choose gain to copy into firmware
useDiscreteGain = false;
if useDiscreteGain
    K = Kd;
else
    K = Kc;
end

%% ESO preview matching firmware PitchEso_UpdateGains()
eso_wn = sqrt(max(K(1) / J, 0.0));
eso_sigma = max((Bv + K(2)) / (2.0 * J), 0.0);
eso_w_cl = max(eso_wn, eso_sigma);
eso_w0_min = max(pitch_up_eso_w0_min_rad_s, 0.0);
eso_w0_max = max(pitch_up_eso_w0_max_rad_s, 0.0);
if eso_w0_max > 0.0 && eso_w0_min > eso_w0_max
    eso_w0_min = eso_w0_max;
end
eso_w0 = pitch_up_eso_alpha * eso_w_cl;
if eso_w0 < eso_w0_min
    eso_w0 = eso_w0_min;
end
if eso_w0_max > 0.0 && eso_w0 > eso_w0_max
    eso_w0 = eso_w0_max;
end
eso_b0 = 1.0 / J;
eso_beta1 = 3.0 * eso_w0;
eso_beta2 = 3.0 * eso_w0 * eso_w0;
eso_beta3 = eso_w0 * eso_w0 * eso_w0;

fprintf('\n=== Pitch-up LQR result ===\n');
fprintf('Continuous Kc = [%.6ff, %.6ff]\n', Kc(1), Kc(2));
fprintf('Discrete   Kd = [%.6ff, %.6ff]\n', Kd(1), Kd(2));
fprintf('Selected   K = [%.6ff, %.6ff]\n', K(1), K(2));

fprintf('\nCopy to application/gimbal/gimbal.c:\n');
fprintf('#define PITCH_UP_LQR_K_THETA                %.6ff\n', K(1));
fprintf('#define PITCH_UP_LQR_K_OMEGA                %.6ff\n', K(2));
fprintf('#define PITCH_UP_ESO_TRIAL_ENABLE           %dU\n', pitch_up_eso_trial_enable);
fprintf('#define PITCH_UP_MODEL_J_KG_M2              %.9gf\n', J);
fprintf('#define PITCH_UP_MODEL_B_NMS_RAD            %.9gf\n', Bv);
fprintf('#define PITCH_UP_ESO_ENABLE                 %dU\n', pitch_up_eso_enable);
fprintf('#define PITCH_UP_ESO_ALPHA                  %.9gf\n', pitch_up_eso_alpha);
fprintf('#define PITCH_UP_ESO_COMP_GAIN              %.9gf\n', pitch_up_eso_comp_gain);
fprintf('#define PITCH_UP_ESO_COMP_LIMIT_NM          %.9gf\n', pitch_up_eso_comp_limit_nm);
fprintf('#define PITCH_UP_ESO_W0_MIN_RAD_S           %.9gf\n', pitch_up_eso_w0_min_rad_s);
fprintf('#define PITCH_UP_ESO_W0_MAX_RAD_S           %.9gf\n', pitch_up_eso_w0_max_rad_s);

fprintf('\nESO preview matching firmware calculation:\n');
fprintf('b0     = %.9gf\n', eso_b0);
fprintf('w_cl   = %.9gf rad/s\n', eso_w_cl);
fprintf('w0     = %.9gf rad/s\n', eso_w0);
fprintf('beta1  = %.9gf\n', eso_beta1);
fprintf('beta2  = %.9gf\n', eso_beta2);
fprintf('beta3  = %.9gf\n', eso_beta3);

fprintf('\nClosed-loop poles:\n');
fprintf('Continuous poles:\n');
disp(poles_c);
fprintf('Discrete poles:\n');
disp(poles_d);

%% Simple saturated step simulation
torqueLimit = 7.0;  % match PITCH_UP_TORQUE_LIMIT_NM
refStep = 0.2;      % rad
Tend = 1.5;         % s
N = round(Tend / Ts);

x = [-refStep; 0];  % measure starts at 0, ref steps to +refStep
log_t = (0:N-1)' * Ts;
log_x = zeros(N, 2);
log_u = zeros(N, 1);

for i = 1:N
    u = -K * x;
    u = min(max(u, -torqueLimit), torqueLimit);

    xdot = A * x + B * u;
    x = x + Ts * xdot;

    log_x(i, :) = x.';
    log_u(i) = u;
end

if enablePlot
    figure('Name', 'Pitch-up LQR saturated step');
    subplot(2, 1, 1);
    plot(log_t, log_x(:, 1), 'LineWidth', 1.2);
    grid on;
    ylabel('e_\theta = measure-ref (rad)');

    subplot(2, 1, 2);
    plot(log_t, log_u, 'LineWidth', 1.2);
    grid on;
    ylabel('\tau_{lqr} (Nm)');
    xlabel('Time (s)');
end
