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

%% Latest identified plant parameters
% Keep these values synchronized with the MATLAB candidate model.
% They are intentionally not written to lqr_eso.c until the firmware defaults
% are updated after field validation.
% Non-auto yaw currently aliases the self-aim yaw defaults:
%   YAW_J_DEFAULT = YAW_AUTO_J_DEFAULT
%   YAW_B_DEFAULT = YAW_AUTO_B_DEFAULT
%   LQR_K_* = LQR_AUTO_K_*
%   YAW_TAU_COULOMB_DEFAULT = YAW_AUTO_TAU_COULOMB_DEFAULT
% Rollback values before sync:
%   J=0.033740, Bv=0.388210, Ktheta=27.7195954, Komega=1.33991836,
%   Coulomb=0.346952, smooth=0.15.
J  = 0.0686165392;     % y1/y2 constrained-fit yaw J candidate, kg*m^2
Bv = 0.226753756;      % y1/y2 constrained-fit yaw B candidate, N*m*s/rad
Ts = 0.001;            % YAW_TS_DEFAULT, s
T_limit = 7.0;         % CTRL_T_LIMIT_DEFAULT / DM torque limit, N*m
coulombFcNm = 0.2942341095;  % y1 stage-1 low-speed Coulomb candidate, N*m
coulombSmoothOmega = 0.150000006;  % YAW_COULOMB_SMOOTH_OMEGA_DEFAULT, rad/s

%% Tuning section
% Increase Q_theta for tighter angle tracking.
% Increase Q_omega for more damping / less overshoot.
% Increase R_tau for softer torque and slower response.
% The printed K comes only from dlqr(Ad, Bd, Q, R), so changing Q/R below
% changes g_lqr_k_theta and g_lqr_k_omega in the output.
% Current values reproduce the y1/y2 candidate:
%   LQR_K_THETA_DEFAULT = 111.405487
%   LQR_K_OMEGA_DEFAULT = 4.60135192
Q_theta = 400;
Q_omega = 0.5;
R_tau   = 1.0;

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
[Kd, ~, ~] = dlqr(Ad, Bd, Q, R);

%% Print K only
fprintf('K = [%.9g, %.9g]\n', Kd(1), Kd(2));
