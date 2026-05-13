clc;
clear;
close all;

%% DM-J4310 yaw LQRESO LQR gain tuning
% This script matches the current firmware implementation in:
%   Infantry_RMUC_2026/modules/lqr_eso/lqr_eso.c
%
% Firmware yaw state and units:
%   x = [theta; omega]
%   theta: rad, continuous/unwrapped yaw angle
%   omega: rad/s
%   u:     N*m MIT torque command
%
% Firmware control law:
%   e_theta = z1 - theta_ref_planned
%   e_omega = z2 - omega_ref_planned
%   tau_ff  = J * alpha_ref + B * omega_ref
%   tau_lqr = tau_ff - Ktheta * e_theta - Komega * e_omega
%
% Therefore the MATLAB dlqr gain Kd maps directly to:
%   g_lqr_k_theta = Kd(1)
%   g_lqr_k_omega = Kd(2)
%
% Field tuning workflow:
%   1) Change Q_theta, Q_omega and R_tau in the tuning section below.
%   2) Run this script in MATLAB.
%   3) Copy g_lqr_k_theta and g_lqr_k_omega into the debug watch.
%   4) Test pure LQR first with g_eso_comp_gain reduced or set to 0.

%% Firmware-synchronized plant parameters
% Keep these values synchronized with lqr_eso.c defaults.
J  = 0.0491133;        % YAW_J_DEFAULT, kg*m^2
Bv = 0.0176839;        % YAW_B_DEFAULT, N*m*s/rad
Ts = 0.001;            % YAW_TS_DEFAULT, s
T_limit = 7.0;         % CTRL_T_LIMIT_DEFAULT / DM torque limit, N*m

%% Tuning section
% Increase Q_theta for tighter angle tracking.
% Increase Q_omega for more damping / less overshoot.
% Increase R_tau for softer torque and slower response.
% By default the printed K comes from dlqr(Ad, Bd, Q, R), so changing Q/R
% below changes g_lqr_k_theta and g_lqr_k_omega in the output.
Q_theta = 40;
Q_omega = 0.05;
R_tau   = 0.05;
use_manual_gain = false;        % Set true only to lock the firmware K below.
manual_Ktheta = 78.58128;       % LQR_K_THETA_DEFAULT
manual_Komega = 3.32202;        % LQR_K_OMEGA_DEFAULT

% Offline step sanity check. The LQR gain itself is designed from the linear
% plant. Firmware nonlinear assists below are printed for watch/debug sync.
step_ref_deg = 10.0;
sim_time_s = 2.0;
settle_band_ratio = 0.02;
show_plots = false;      % Set true only when you want an offline response plot.

% Firmware nonlinear assist/default values. These are not part of dlqr(),
% but they affect the real board response around static friction and target hold.
ctrl_angle_deadband_rad = 0.0;   % CTRL_ANGLE_DEADBAND_RAD_DEFAULT
ctrl_vel_deadband = 0.03;        % CTRL_VEL_DEADBAND_DEFAULT
ctrl_min_torque_comp = 0.40;     % CTRL_MIN_TORQUE_COMP_DEFAULT
eso_comp_gain = 0.10;            % ESO_COMP_GAIN_DEFAULT, hard-limited to <= 0.20
eso_comp_limit = 0.10;           % ESO_COMP_LIMIT_DEFAULT, hard-limited to <= 0.15

% Firmware ESO preview values. These only preview what firmware calculates.
eso_alpha = 1.10;      % YAW_ESO_ALPHA_DEFAULT

%% Continuous plant model: J*theta_ddot + B*theta_dot = tau
A = [0,       1;
     0, -Bv / J];
Bmat = [0;
        1 / J];
C = [1, 0];
D = 0;

Q = diag([Q_theta, Q_omega]);
R = R_tau;

if exist('ss', 'file') ~= 2 || exist('c2d', 'file') ~= 2 || exist('dlqr', 'file') ~= 2
    error(['Control System Toolbox functions ss(), c2d() and dlqr() are required ', ...
           'for this firmware-matched tuning script.']);
end

sys_c = ss(A, Bmat, C, D);

%% Discrete model used for dlqr
sys_d = c2d(sys_c, Ts, 'zoh');
Ad = sys_d.A;
Bd = sys_d.B;

%% Discrete LQR gain
if use_manual_gain
    Kd = [manual_Ktheta, manual_Komega];
    P = NaN(2);
    eig_cl = eig(Ad - Bd * Kd);
else
    [Kd, P, eig_cl] = dlqr(Ad, Bd, Q, R);
end
Ktheta = Kd(1);
Komega = Kd(2);

%% Match current firmware ESO gain calculation
b0 = 1 / J;
w0_raw = eso_alpha * sqrt(b0 * abs(Ktheta));
w0 = w0_raw;
beta1 = 3 * w0;
beta2 = 3 * w0^2;
beta3 = w0^3;

%% Offline step sanity check
t = 0:Ts:sim_time_s;
theta_ref = deg2rad(step_ref_deg);
x = zeros(2, numel(t));
u = zeros(1, numel(t));
theta_ref_vec = theta_ref * ones(1, numel(t));

for k = 1:numel(t)-1
    % Same sign convention as firmware after removing tau_ff:
    % tau_lqr = -Ktheta * (theta - theta_ref) - Komega * (omega - 0)
    e = x(:, k) - [theta_ref; 0];
    u(k) = -Kd * e;
    u(k) = min(max(u(k), -T_limit), T_limit);
    x(:, k+1) = Ad * x(:, k) + Bd * u(k);
end
u(end) = u(end-1);

theta_deg = rad2deg(x(1, :));
omega_rad_s = x(2, :);
theta_err_deg = theta_deg - step_ref_deg;
settle_band_deg = settle_band_ratio * max(abs(step_ref_deg), eps);
settle_time = findSettlingTime(t, theta_err_deg, settle_band_deg);
step_dir = sign(step_ref_deg);
if step_dir == 0
    step_dir = 1;
end
overshoot_deg = max(0, max(step_dir * theta_deg) - abs(step_ref_deg));
final_error_deg = theta_err_deg(end);
max_torque = max(abs(u));
torque_limit_ratio = max_torque / T_limit;
sat_sample_count = nnz(abs(u) >= (T_limit - 1e-9));

%% Print results
fprintf('================ DM-J4310 yaw LQRESO LQR tuning ================\n');
fprintf('Firmware model: J*theta_ddot + B*theta_dot = tau\n');
fprintf('J       = %.9gf kg*m^2\n', J);
fprintf('B       = %.9gf N*m*s/rad\n', Bv);
fprintf('Ts      = %.9gf s\n', Ts);
fprintf('T_limit = %.9gf N*m\n', T_limit);

fprintf('\nFirmware nonlinear assist defaults:\n');
fprintf('g_ctrl_angle_deadband_rad = %.9gf;\n', ctrl_angle_deadband_rad);
fprintf('g_ctrl_vel_deadband = %.9gf;\n', ctrl_vel_deadband);
fprintf('g_ctrl_min_torque_comp = %.9gf;\n', ctrl_min_torque_comp);
fprintf('g_eso_comp_gain = %.9gf;\n', eso_comp_gain);
fprintf('g_eso_comp_limit = %.9gf;\n', eso_comp_limit);

fprintf('\nContinuous model:\n');
fprintf('A =\n'); disp(A);
fprintf('B =\n'); disp(Bmat);

fprintf('Discrete model:\n');
fprintf('Ad =\n'); disp(Ad);
fprintf('Bd =\n'); disp(Bd);

fprintf('LQR weights:\n');
fprintf('Q = diag([%.9g, %.9g])\n', Q_theta, Q_omega);
fprintf('R = %.9g\n', R_tau);
fprintf('manual gain mode = %d\n', use_manual_gain);
fprintf('manual firmware K = [%.9gf, %.9gf]\n', manual_Ktheta, manual_Komega);

fprintf('\nKd = [Ktheta, Komega] = [%.9gf, %.9gf]\n', Ktheta, Komega);
fprintf('Closed-loop poles:\n'); disp(eig_cl.');
fprintf('Riccati P:\n'); disp(P);

fprintf('\nCopy to debug watch:\n');
fprintf('g_lqr_k_theta = %.9gf;\n', Ktheta);
fprintf('g_lqr_k_omega = %.9gf;\n', Komega);

fprintf('\nOptional firmware default macros:\n');
fprintf('#define YAW_J_DEFAULT                       %.9g%s\n', J, 'f');
fprintf('#define YAW_B_DEFAULT                       %.9g%s\n', Bv, 'f');
fprintf('#define LQR_K_THETA_DEFAULT                 %.9g%s\n', Ktheta, 'f');
fprintf('#define LQR_K_OMEGA_DEFAULT                 %.9g%s\n', Komega, 'f');
fprintf('#define CTRL_MIN_TORQUE_COMP_DEFAULT        %.9g%s\n', ctrl_min_torque_comp, 'f');

fprintf('\nESO preview matching current firmware calculation:\n');
fprintf('b0     = %.9gf\n', b0);
fprintf('w0_raw = %.9gf\n', w0_raw);
fprintf('w0     = %.9gf   (no clamp in current firmware)\n', w0);
fprintf('beta1  = %.9gf\n', beta1);
fprintf('beta2  = %.9gf\n', beta2);
fprintf('beta3  = %.9gf\n', beta3);

fprintf('\nFirmware ESO/debug variables to tune online:\n');
fprintf('g_yaw_eso_alpha = %.9gf;\n', eso_alpha);
fprintf('g_eso_comp_gain = %.9gf;\n', eso_comp_gain);
fprintf('g_eso_comp_limit = %.9gf;\n', eso_comp_limit);

fprintf('\nSanity step: %.3f deg, torque limit %.3f Nm\n', step_ref_deg, T_limit);
fprintf('max torque        = %.6f Nm (%.2f%% of limit)\n', max_torque, 100 * torque_limit_ratio);
fprintf('limit samples     = %d / %d\n', sat_sample_count, numel(u));
fprintf('overshoot         = %.6f deg\n', overshoot_deg);
fprintf('final error       = %.6f deg\n', final_error_deg);
fprintf('settle band       = +/- %.6f deg\n', settle_band_deg);
fprintf('settle time       = %.6f s\n', settle_time);

%% Optional plots
if show_plots
    figure('Name', 'Yaw LQRESO LQR tuning sanity check');

    subplot(3,1,1);
    plot(t, theta_deg, 'LineWidth', 1.2); hold on;
    plot(t, rad2deg(theta_ref_vec), '--', 'LineWidth', 1.0);
    yline(step_ref_deg + settle_band_deg, ':');
    yline(step_ref_deg - settle_band_deg, ':');
    grid on;
    ylabel('theta (deg)');
    title('Closed-loop step check without ESO compensation');
    legend('theta', 'theta ref', 'settle band', 'Location', 'best');

    subplot(3,1,2);
    plot(t, omega_rad_s, 'LineWidth', 1.2);
    grid on;
    ylabel('omega (rad/s)');

    subplot(3,1,3);
    plot(t, u, 'LineWidth', 1.2); hold on;
    yline(T_limit, '--r');
    yline(-T_limit, '--r');
    grid on;
    ylabel('tau (N*m)');
    xlabel('time (s)');
end

%% Notes
% If the real machine oscillates:
%   1) reduce Q_theta or increase R_tau;
%   2) increase Q_omega carefully for more damping;
%   3) reduce g_yaw_eso_alpha;
%   4) reduce g_eso_comp_gain or set it to 0 to test pure LQR.
%
% If response is too soft:
%   1) increase Q_theta carefully;
%   2) reduce R_tau;
%   3) enable the firmware reference planner instead of using a hard step.

function settle_time = findSettlingTime(t, err, band)
% Return the first time after which the response stays inside +/-band.
    settle_time = NaN;
    inside = abs(err) <= band;

    for idx = 1:numel(t)
        if all(inside(idx:end))
            settle_time = t(idx);
            return;
        end
    end
end
