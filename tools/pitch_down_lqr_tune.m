%% pitch_down_lqr_tune.m
% Compute pitch-down LQR gain from the firmware model and Q/R weights.

clear; clc;

%% Firmware-synchronized model
J = 0.08984;      % kg*m^2, PITCH_DOWN_MODEL_J_KG_M2
Bv = 0.0;         % N*m*s/rad, PITCH_DOWN_MODEL_B_NMS_RAD

%% Tunable LQR weights
q_theta = 5000.0; % PITCH_DOWN_MODEL_Q_THETA
q_omega = 10;    % PITCH_DOWN_MODEL_Q_OMEGA
r_tau = 1.0;      % PITCH_DOWN_MODEL_R_TORQUE

K = pitch_down_analytic_care_gain(J, Bv, q_theta, q_omega, r_tau);

fprintf('K = [%.6f %.6f]\n', K(1), K(2));

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
