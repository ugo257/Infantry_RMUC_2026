%% double_pitch_lqr_tune.m
% Generate the fixed 2x4 firmware K matrix for the current double-pitch LQR.
%
% Firmware path:
%   modules/double_pitch_lqr/double_pitch_lqr.c
%
% Firmware state and sign convention:
%   x = [e_theta; e_phi; omega_big; omega_small]
%   e_theta = theta_rad - theta_ref_rad
%   e_phi   = phi_rad   - phi_ref_rad
%   omega_big   = theta_big_dot_radps
%   omega_small = phi_rel_dot_radps
%
% Firmware control law:
%   v_big   = -K(1,:) * x
%   v_small = -K(2,:) * x
%   tau_fb  = M * [v_big; v_small]
%   tau_cmd = tau_fb + tau_gravity
%
% This script solves LQR with physical motor torque as the input, then
% converts the torque-feedback gain into the firmware acceleration gain:
%   K_tau      = lqr(A, B_tau, Q, R)
%   K_firmware = M \ K_tau
%
% Therefore the printed K_firmware drops directly into g_double_pitch_k_live.
% Gravity, friction, Coriolis terms, torque limits and slew limits are not
% part of the Riccati solve. They are included only in the sanity simulation.

clear; clc; close all;

%% Firmware-synchronized model constants
% Keep these values synchronized with modules/double_pitch_lqr/double_pitch_lqr.c.
A0 = 0.0314;             % DOUBLE_PITCH_MODEL_A0
AC = 0.0000;             % DOUBLE_PITCH_MODEL_AC
AS = 0.0000;             % DOUBLE_PITCH_MODEL_AS
B0 = -0.0057;            % DOUBLE_PITCH_MODEL_B0, unchanged: pitch-up data did not excite theta_ddot enough
BC = 0.0000;             % DOUBLE_PITCH_MODEL_BC
BS = 0.0000;             % DOUBLE_PITCH_MODEL_BS
C0 = 0.011526;           % DOUBLE_PITCH_MODEL_C0

GB1 = 1.0666;            % DOUBLE_PITCH_MODEL_GB1
GB2 = 0.0000;            % DOUBLE_PITCH_MODEL_GB2
GP1 = 0.373492;          % DOUBLE_PITCH_MODEL_GP1
GP2 = -0.929636;         % DOUBLE_PITCH_MODEL_GP2

FVB = 0.0000;            % DOUBLE_PITCH_MODEL_FVB, debug only in current firmware
FCB = 0.0000;            % DOUBLE_PITCH_MODEL_FCB, debug only in current firmware
FVS = 0.063380;          % DOUBLE_PITCH_MODEL_FVS
FCS = 0.333765;          % DOUBLE_PITCH_MODEL_FCS
OMEGA0 = 0.0500;         % DOUBLE_PITCH_MODEL_OMEGA0, debug only in current firmware

Ts = 0.001;              % nominal task period, s

% Firmware torque limits from application/gimbal/gimbal.c.
useHoleModeLimits = false;
torqueLimitBigNormal = 6.0;       % PITCH_DOWN_LQR_TORQUE_LIMIT_NM
torqueLimitBigHole = 2.0;         % PITCH_DOWN_HOLE_TORQUE_LIMIT_NM
torqueLimitSmall = 7.0;           % PITCH_UP_TORQUE_LIMIT_NM
torqueSlewRateBigNormal = 700.0;  % PITCH_DOWN_TORQUE_SLEW_RATE_NM_S
torqueSlewRateBigHole = 700.0;    % PITCH_DOWN_HOLE_TORQUE_SLEW_RATE_NM_S
torqueSlewRateSmall = 1000.0;     % PITCH_UP_TORQUE_SLEW_RATE_NM_S
torqueLimitBig = selectScalar(useHoleModeLimits, torqueLimitBigHole, torqueLimitBigNormal);
torqueSlewRateBig = selectScalar(useHoleModeLimits, torqueSlewRateBigHole, torqueSlewRateBigNormal);
torqueSlewEnable = true;

%% Linearization point
% Current firmware has AC/AS/BC/BS all zero, so M does not actually depend on
% phi. Keep phi0 here so this script still matches the same formula if those
% model terms are re-enabled later.
phi0 = -1.65;            % rad, representative relative pitch angle
theta0 = 0.0;            % rad, used only by the nonlinear sanity simulation

%% Tunable QR matrices
% State is x = [e_theta; e_phi; omega_big; omega_small].
%
% Q(1,1): theta/down absolute angle error. Larger means down follows its
%         target harder and holds the big joint more stiffly.
% Q(2,2): phi/up relative angle error. Larger means the up/relative joint
%         catches up harder and feels less soft.
% Q(3,3): down angular speed. Larger adds damping to down motion and reduces
%         overshoot, but can increase braking torque.
% Q(4,4): up relative angular speed. Larger adds damping to the relative/up
%         motion and reduces oscillation.
%
% Off-diagonal Q entries couple states. Start with zero off-diagonals unless
% you intentionally want theta/phi or speed coupling.
q_theta = 80.0;
q_phi = 40.0;
q_omega_big = 1.0;
q_omega_small = 1.0;
q_theta_phi = 0.0;
q_omega_couple = 0.0;

Q = [q_theta,      q_theta_phi, 0.0,            0.0;
     q_theta_phi,  q_phi,       0.0,            0.0;
     0.0,          0.0,         q_omega_big,    q_omega_couple;
     0.0,          0.0,         q_omega_couple, q_omega_small];

% R penalizes physical motor torque:
% R(1,1): down/big motor torque cost. Larger makes down softer.
% R(2,2): up/small motor torque cost. Larger makes up softer.
% R(1,2): coupled torque cost. Keep zero for normal tuning.
r_tau_big = 0.8;
r_tau_small = 0.5;
r_tau_couple = 0.0;

R = [r_tau_big,    r_tau_couple;
     r_tau_couple, r_tau_small];

%% Current firmware K for comparison
currentK = [343.2,  -149.0,  24.0,  -17.0;
           -638.6, 1777.2, -44.7, 203.1];

% Set true only when you want the script to print the current firmware K
% instead of the QR-derived K.
useManualK = false;
manualK = currentK;

%% Offline sanity-step settings
enablePlot = false;
simTimeS = 1.5;
thetaStep = 0.10;    % rad, target step for theta
phiStep = 0.10;      % rad, target step for phi

%% Build firmware-matched linear model
sinPhi = sin(phi0);
cosPhi = cos(phi0);

m11 = A0 + AC * cosPhi + AS * sinPhi;
m12 = B0 + BC * cosPhi + BS * sinPhi;
m22 = C0;
M = [m11, m12;
     m12, m22];

if rcond(M) < 1.0e-8
    error('Mass matrix M is near singular. Check model constants.');
end

% Physical plant:
%   qdot = omega
%   omega_dot = M \ tau
A = [0, 0, 1, 0;
     0, 0, 0, 1;
     0, 0, 0, 0;
     0, 0, 0, 0];
Btau = [zeros(2, 2);
        M \ eye(2)];

validateLqrWeights(Q, R);
Q = 0.5 * (Q + Q.');
R = 0.5 * (R + R.');

if exist('lqr', 'file') ~= 2
    error('Control System Toolbox lqr() is required for this tuning script.');
end

ctrbMat = [Btau, A * Btau, A^2 * Btau, A^3 * Btau];
if rank(ctrbMat) < size(A, 1)
    error('Double-pitch plant is not controllable. Check M and Btau.');
end

%% Solve LQR and convert to firmware K
if useManualK
    Kfirmware = manualK;
    Ktau = M * Kfirmware;
    P = NaN(4);
    gainSource = 'manualK';
else
    [Ktau, P, ~] = lqr(A, Btau, Q, R);
    Kfirmware = M \ Ktau;
    gainSource = 'lqr(A, Btau, Q, R), converted by Kfirmware = M \\ Ktau';
end

closedLoopPoles = eig(A - Btau * Ktau);
KtauCurrent = M * currentK;
KtauFirmware = M * Kfirmware;

%% Firmware-style saturated sanity simulation
N = max(2, round(simTimeS / Ts) + 1);
t = (0:N-1) * Ts;
x = zeros(4, N);
tau = zeros(2, N);
tauRaw = zeros(2, N);
vCmd = zeros(2, N);
limitFlag = false(2, N);
ref = [thetaStep; phiStep; 0.0; 0.0];
tauLast = [0.0; 0.0];

for k = 1:N-1
    e = x(:, k) - ref;
    v = -Kfirmware * e;
    tauFb = M * v;
    tauG = gravityTorque(theta0 + x(1, k), phi0 + x(2, k), ...
                         GB1, GB2, GP1, GP2);
    tauRaw(:, k) = tauFb + tauG;
    tauLimited = [clampScalar(tauRaw(1, k), -torqueLimitBig, torqueLimitBig);
                  clampScalar(tauRaw(2, k), -torqueLimitSmall, torqueLimitSmall)];

    if torqueSlewEnable
        maxDelta = [torqueSlewRateBig; torqueSlewRateSmall] * Ts;
        tauLimited = [clampScalar(tauLimited(1), tauLast(1) - maxDelta(1), tauLast(1) + maxDelta(1));
                      clampScalar(tauLimited(2), tauLast(2) - maxDelta(2), tauLast(2) + maxDelta(2))];
    end

    tau(:, k) = tauLimited;
    limitFlag(:, k) = abs(tau(:, k) - tauRaw(:, k)) > 1.0e-9;
    tauLast = tau(:, k);
    vCmd(:, k) = v;

    xdot = A * x(:, k) + Btau * (tau(:, k) - tauG);
    x(:, k + 1) = x(:, k) + Ts * xdot;
end

tau(:, end) = tau(:, end - 1);
tauRaw(:, end) = tauRaw(:, end - 1);
vCmd(:, end) = vCmd(:, end - 1);
limitFlag(:, end) = limitFlag(:, end - 1);

%% Print results
printMatrixC(Kfirmware);

%% Optional plots
if enablePlot
    figure('Name', 'Double-pitch QR-LQR sanity simulation');

    subplot(3, 1, 1);
    plot(t, x(1, :), 'LineWidth', 1.2); hold on;
    plot(t, x(2, :), 'LineWidth', 1.2);
    yline(thetaStep, '--');
    yline(phiStep, '--');
    grid on;
    ylabel('angle error state (rad)');
    legend('theta', 'phi', 'theta ref', 'phi ref', 'Location', 'best');

    subplot(3, 1, 2);
    plot(t, x(3, :), 'LineWidth', 1.2); hold on;
    plot(t, x(4, :), 'LineWidth', 1.2);
    grid on;
    ylabel('speed state (rad/s)');
    legend('omega big', 'omega small', 'Location', 'best');

    subplot(3, 1, 3);
    plot(t, tau(1, :), 'LineWidth', 1.2); hold on;
    plot(t, tau(2, :), 'LineWidth', 1.2);
    yline(torqueLimitBig, '--r');
    yline(-torqueLimitBig, '--r');
    yline(torqueLimitSmall, ':r');
    yline(-torqueLimitSmall, ':r');
    grid on;
    ylabel('tau (Nm)');
    xlabel('time (s)');
    legend('tau big', 'tau small', 'big limit', 'small limit', 'Location', 'best');
end

function validateLqrWeights(Q, R)
    if ~isequal(size(Q), [4, 4])
        error('Q must be 4x4 for x = [e_theta; e_phi; omega_big; omega_small].');
    end
    if ~isequal(size(R), [2, 2])
        error('R must be 2x2 for u = [tau_big; tau_small].');
    end
    if any(~isfinite(Q(:))) || any(~isfinite(R(:)))
        error('Q and R must contain finite numeric values.');
    end
    if max(max(abs(Q - Q.'))) > 1.0e-9
        error('Q must be symmetric.');
    end
    if max(max(abs(R - R.'))) > 1.0e-9
        error('R must be symmetric.');
    end
    if min(eig(Q)) < -1.0e-9
        error('Q must be positive semidefinite.');
    end
    if min(eig(R)) <= 1.0e-9
        error('R must be positive definite.');
    end
end

function y = clampScalar(x, lower, upper)
    y = min(max(x, lower), upper);
end

function y = selectScalar(condition, trueValue, falseValue)
    if condition
        y = trueValue;
    else
        y = falseValue;
    end
end

function tauG = gravityTorque(theta, phi, GB1, GB2, GP1, GP2)
    thetaPlusPhi = theta + phi;
    tauBigG = GB1 * sin(theta) + GB2 * cos(theta) + ...
              GP1 * sin(thetaPlusPhi) + GP2 * cos(thetaPlusPhi);
    tauSmallG = GP1 * sin(thetaPlusPhi) + GP2 * cos(thetaPlusPhi);
    tauG = [tauBigG; tauSmallG];
end

function printMatrixC(K)
    fprintf('[%.9g, %.9g, %.9g, %.9g;\n', K(1, 1), K(1, 2), K(1, 3), K(1, 4));
    fprintf(' %.9g, %.9g, %.9g, %.9g]\n', K(2, 1), K(2, 2), K(2, 3), K(2, 4));
end
