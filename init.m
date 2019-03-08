clear;
clc;

%% Physical model (joint and load)

% joint mechanical parameters
% *** for SD test joint ***
% n_HD = 101;             % -, transmission ratio (motor speed)/(link speed)
% Jm = 1;                 % kgm^2, motor rotor inertia, motor side
% J_HD = 0;               % kgm^2, harmonic driver inertia, link side
% J_joint = Jm*n_HD^2 J_HD;    % kgm^2, combined joint inertia scaled to link side
% K_el = 6900;            % Nm/rad, torque sensor stiffness
% D_el = 0;                % Nms/rad, torque sensor damping
% D_visc = 0;              % Nms/rad
% F_c = 0;                 % Nm, coulombic friction in motor

% *** for LBR joint 1 (from beasty parameter files)
J_joint = 4.155;        % kgm^2
K_el = 6900;            % Nm/rad, torque sensor stiffness
D_el = 0;               % Nms/rad, torque sensor damping
D_visc = 20;            % Nms/rad

% LuGre friction model
% steady state friction function:
% F_ss(v) = (Fc + (Fs-Fc)exp(-(v/vs)^alpha))*sgn(v) + sigma2*v
% dynamics:
% sigma0 = microstiffness, sigma1 = microdamping
% state space equations:
% g(v) = Fc + (Fs-Fc)exp(-(v/vs)^alpha)
% dz = v - sigma0*abs(v)/g(v)*z
% F = sigma0*z + sigma1*dz + sigma2*v
F_c = 35;               % Nm, coulombic friction (35)
F_s = 40;               % Nm, coulombic friction (40)
assert(F_s >= F_c);     % static friction must be greater or equal to coulombic friction
sigma0 = 2000;          % Nm/rad, stiffness parameter (torque/bristle deflection z)
sigma1 = 200;           % Nms/rad, microdamping
sigma2 = D_visc;        % Nms/rad, viscous friction
vs = 0.1;               % rad/s, speed for friction to reach 68% of the way from Fs to Fc
alpha = 1.0;            % - , v/vs exponent

% load parameters
J_load = 1.2252;        % kgm^2, load inertia, link side
% J_load = J_load*(1+0.4);
% J_load = 0.01;

%% Controller parameters

T_cur = 2e-3;           % s, current control closed-loop time constant

% controller parameters
T_signal_LPF = 2e-3;    % s, signal filter time constant
tau_max_ctrl = 180;     % Nm, maximum torque

% controller gains for zero force control mode
Kp_zf = 0;
Kd_zf = 0;
Ks_zf = 0;
Kt_zf = 0;
K__zf = 0;     % not used in beasty sim controller
Ki_zf = 0;     % not used in beasty sim controller

% controller gains for torque control mode
Kp_trq = 25;
Kd_trq = 10;
Ks_trq = 0.04;
Kt_trq = 9.10251;
K__trq = 0;     % not used in beasty sim controller
Ki_trq = 0;     % not used in beasty sim controller

% controller gains for position control mode
Kp_pos = 6124.290039;
Kd_pos = 410.747040;   
Ks_pos = 0.006793;
Kt_pos = 0.126640;
K__pos = 0;     % not used in beasty sim controller
Ki_pos = 0;     % not used in beasty sim controller

% equivalence between B_theta and Kt_trq
% From DLR papers: -Kt_trq = (1-J_joint/B_theta)
% where B_theta is the desired apparent rotor inertia
B_theta = J_joint/(1+Kt_trq);

% friction observer
L_fricObs = 1000;     % 1/s

% Collision detection
K0_colli = 1000;         % gain of momentum observer
threshold_colli = 5;    % threshold to distinguish collision and noise