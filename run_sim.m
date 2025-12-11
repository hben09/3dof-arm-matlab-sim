% RUN_SIM
%
% Main simulation script for the 3-DOF robotic arm
% Runs dynamics, visualizes results, and plots tracking error

clear; clc; close all;

addpath('visualization');

%% 0. Define Robot Parameters
params.m1 = 1.0;  % Waist mass (kg)
params.m2 = 5.0;  % Upper arm mass (kg)
params.m3 = 3.0;  % Forearm mass (kg)

params.a2 = 0.5;  % Upper arm length (m)
params.a3 = 0.5;  % Forearm length (m)

params.g = 9.81;  % Gravitational acceleration (m/s^2)

I_rod2 = (1/12) * params.m2 * params.a2^2;
I_rod3 = (1/12) * params.m3 * params.a3^2;
params.I1 = diag([0.01, 0.01, 0.01]);
params.I2 = diag([0.01, I_rod2, I_rod2]);
params.I3 = diag([0.01, I_rod3, I_rod3]);

%% 1. Define Simulation Settings
t_start = 0;      % Simulation start time (s)
t_end = 5;        % Duration of each segment (s)
tspan = [t_start, t_end];

q1_0 = -0.2;      % Initial waist angle (rad)
q2_0 = 0.0;       % Initial shoulder angle (rad)
q3_0 = -0.2;      % Initial elbow angle (rad)
x0 = [q1_0; q2_0; q3_0; 0; 0; 0];

% Sequence of target end-effector positions [x; y; z] (m)
target_pos = {[0.5; 0.5; 0.5], [0.0; -0.6; 0.4], [-0.5; 0.5; 0.2]};

params.traj_start_time = 0.0;  % Trajectory start time within each segment (s)
params.traj_duration = 4.0;    % Trajectory duration (s)

params.q_start = x0(1:3);
params.pos_start = get_forward_kinematics(x0(1:3), params.a2, params.a3);

%% 2. Define Control Settings
omega_n = 10;     % Natural frequency (rad/s) - higher = faster/stiffer response
zeta = 1;         % Damping ratio - 1 = critically damped
params.kp = omega_n^2;
params.kd = 2*zeta*omega_n;

% 'JOINT' = control joint angles | 'OPERATIONAL' = control end-effector position
params.CONTROL_SPACE = 'OPERATIONAL';

% true = smooth trajectory | false = step input
params.USE_TRAJECTORY = true;

% true = computed torque (cancel B,C,G) | false = PD + gravity compensation
params.USE_INVERSE_DYNAMICS = true;

%% 3. Run Simulation
disp('Running Simulation with ode15s...');

options = odeset('RelTol', 1e-3, 'AbsTol', 1e-3);
t_all = {};
x_all = {};

for i = 1:length(target_pos)
    fprintf('Running segment %d/%d...\n', i, length(target_pos));

    params.pos_target = target_pos{i};
    q_final = get_inverse_kinematics(target_pos{i}(1), target_pos{i}(2), target_pos{i}(3), params.a2, params.a3);
    params.q_target = q_final;

    [t_segment, x_segment] = ode15s(@(t,x) f_arm_dynamics(t, x, params), tspan, x0, options);

    t_all{end+1} = t_segment;
    x_all{end+1} = x_segment;

    x0 = x_segment(end, :);

    params.q_start = x0(1:3)';
    p_start = get_forward_kinematics(x0(1:3)', params.a2, params.a3);
    params.pos_start = p_start;
    params.traj_start_time = t_start;
end

%% 4. Concatenate Segments
t = t_all{1};
x = x_all{1};
for i = 2:length(t_all)
    t_segment = t_all{i};
    x_segment = x_all{i};
    t = [t; t(end) + t_segment(2:end)];
    x = [x; x_segment(2:end,:)];
end
t_end = t(end);

disp('Simulation Complete. Preparing Animation...');

%% 5. Interpolate for Animation
fps = 30;         % Animation frame rate (frames/s)
t_anim = t_start : 1/fps : t_end;
x_anim = interp1(t, x, t_anim);

%% 6. Calculate Ghost Path
ghost_path = calculate_ghost_path(t_anim, target_pos, t_all, x_all, params, q1_0, q2_0, q3_0);

%% 7. Plot Joint Angles
figure('Name', 'Joint Angles');
plot(t, x(:, 1:3), 'LineWidth', 2);
legend('Waist (q1)', 'Shoulder (q2)', 'Elbow (q3)');
xlabel('Time (s)');
ylabel('Angle (rad)');
title('Joint Angles vs Time');
grid on;

%% 8. Animate Robot
visualize_robot(t_anim, x_anim, target_pos, params, t_end, fps, ghost_path);

%% 9. Plot Tracking Error
plot_tracking_error(t_all, x_all, target_pos, params, q1_0, q2_0, q3_0);
