%% yaw_auto_lqr_tune.m
% Tune self-aim yaw LQR feedback gains from QR weights.
%
% Firmware path:
%   modules/lqr_eso/lqr_eso.c, YawLQRESO_Calc()
%
% Self-aim yaw plant:
%   J * theta_ddot + B * theta_dot = tau
%
% The printed K maps directly to:
%   g_yaw_auto_j = J
%   g_yaw_auto_b = B
%   g_lqr_auto_k_theta = K(1)
%   g_lqr_auto_k_omega = K(2)
%
% Plant/Coulomb constants below are synchronized with the latest yawww.csv
% g_yaw_model_ident fitting candidate. Stage 2 fits J/B from dynamic chirp
% data, while stage 1 fixes Coulomb friction from low-speed motion.
% These values match the current lqr_eso.c firmware defaults.

clear; clc;

%% Latest identified self-aim yaw model candidate
J = 0.0422563489;       % yawww.csv stage-2 motor_tor_nm fit, kg*m^2
Bv = 0.286217284;       % yawww.csv stage-2 motor_tor_nm fit, N*m*s/rad
Ts = 0.001;             % YAW_TS_DEFAULT, s
esoAlpha = 1.10;        % YAW_ESO_ALPHA_DEFAULT
coulombFcNm = 0.356476291;   % yawww.csv dynamic Coulomb candidate, N*m
coulombSmoothOmega = 0.150000006;  % YAW_AUTO_COULOMB_SMOOTH_OMEGA_DEFAULT, rad/s

%% Tunable QR matrices
% State is x = [theta_error_rad; omega_error_rad_s].
% These continuous-LQR weights reproduce the yawww.csv firmware candidate:
%   K = [67.6101582, 2.58721444]
Q = [4571.133491829028, 0;
        0, 2.4607726706624833];
R = 1.0;

%% Continuous plant model used for self-aim K
A = [0, 1;
     0, -Bv / J];
Bu = [0;
      1 / J];

validateQrMatrices(Q, R);
Q = 0.5 * (Q + Q.');

if rank([Bu, A * Bu]) < size(A, 1)
    error('Self-aim yaw plant is not controllable. Check J and Bv.');
end

if exist('lqr', 'file') ~= 2
    error('Control System Toolbox function lqr() is required.');
end

%% LQR gain from current Q/R
[K, ~, ~] = lqr(A, Bu, Q, R);

%% Print K only
fprintf('K = [%.9g, %.9g]\n', K(1), K(2));

function validateQrMatrices(Q, R)
    if ~isequal(size(Q), [2, 2])
        error('Q must be a 2x2 matrix for x = [theta_error; omega_error].');
    end
    if ~isscalar(R)
        error('R must be a scalar / 1x1 matrix for yaw torque input.');
    end
    if any(~isfinite(Q(:))) || ~isfinite(R)
        error('Q and R must contain finite numeric values.');
    end
    if max(max(abs(Q - Q.'))) > 1e-9
        error('Q must be symmetric. Use Q = 0.5 * (Q + Q.'') if needed.');
    end
    if min(eig(Q)) < -1e-9
        error('Q must be positive semidefinite.');
    end
    if R <= 0.0
        error('R must be positive.');
    end
end
