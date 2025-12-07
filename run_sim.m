% Benjamin Hsu
% ME500 Final Project
%
% SCRIPT: run_sim.m
% PURPOSE: Run the dynamic simulation of the 3-DOF Robotic Arm

clear; clc; close all;

%% 1. Define Robot Parameters
% --- Mass ---
params.m1 = 1.0;  % Mass of Waist (kg)
params.m2 = 5.0;  % Mass of Upper Arm (kg)
params.m3 = 3.0;  % Mass of Forearm (kg)

% --- Geometry ---
params.a2 = 0.5;  % Length of Upper Arm (m)
params.a3 = 0.5;  % Length of Forearm (m)

% --- Environment ---
params.g  = 9.81; % Gravity (m/s^2)

% --- Inertia ---
% Inertia matrices are modeled as slender rods about their CoM.
I_rod2 = (1/12) * params.m2 * params.a2^2;
I_rod3 = (1/12) * params.m3 * params.a3^2;
params.I1 = diag([0.01, 0.01, 0.01]);      % Waist
params.I2 = diag([0.01, I_rod2, I_rod2]);  % Upper Arm
params.I3 = diag([0.01, I_rod3, I_rod3]);  % Forearm

%% 2. Define Simulation & Control Settings
% --- Simulation Time ---
t_start = 0;
t_end   = 5; % Simulation segment length (s)
tspan   = [t_start, t_end];

% --- Initial Conditions ---
% State vector: x = [q1; q2; q3; dq1; dq2; dq3]
q1_0 = -0.2; % Facing forward
q2_0 = 0.0;  % Shoulder horizontal
q3_0 = -0.2; % Elbow straight
x0 = [q1_0; q2_0; q3_0; 0; 0; 0]; 

% --- Target Configuration ---
% Define a sequence of target positions for the end-effector.
target_pos = {[0.5; 0.5; 0.5], [0.0; -0.6; 0.4], [-0.5; 0.5; 0.5]}; 
params.vel_target = [0; 0; 0]; % Target velocity is always zero.

% --- Trajectory Timing ---
params.traj_start_time = 1.0; 
params.traj_duration = 4.0;

% --- Initial Trajectory Conditions ---
% These are updated at the start of each new segment in the loop.
params.q_start = x0(1:3);
params.pos_start = get_forward_kinematics(x0(1:3), params.a2, params.a3);

% --- Controller Gains ---
omega_n = 10; % Natural frequency (higher = stiffer/faster response)
zeta    = 1;  % Damping ratio (1 = critically damped)
params.kp = omega_n^2;
params.kd = 2*zeta*omega_n;

% --- Control Configuration ---

% 1. Choose Control Space
% 'JOINT'       = Control joint angles (q1, q2, q3)
% 'OPERATIONAL' = Control end-effector position (x, y, z)
params.CONTROL_SPACE = 'JOINT'; 

% 2. Choose Reference Type
% true  = Follow a smooth path (Trajectory Planning)
% false = Jump to target immediately (Step Input)
params.USE_TRAJECTORY = true;

% 3. Choose Dynamics Compensation
% true  = Full Inverse Dynamics (Cancel B, C, G) -> "Computed Torque"
% false = Gravity Compensation Only (Cancel G)   -> "PD Control"
params.USE_INVERSE_DYNAMICS = false;

%% 3. Run Simulation (ODE Solver)
disp('Running Simulation with ode15s...');

options = odeset('RelTol', 1e-3, 'AbsTol', 1e-3);
% Using ode15s because high gains make the system 'stiff'
t_all = {};
x_all = {};

for i = 1:length(target_pos)
    fprintf('Running segment %d/%d...\n', i, length(target_pos));
    
    % Set the current target for this segment
    params.pos_target = target_pos{i};
    q_final = get_inverse_kinematics(target_pos{i}(1), target_pos{i}(2), target_pos{i}(3), params.a2, params.a3);
    params.q_target = q_final;
    
    [t_segment, x_segment] = ode15s(@(t,x) f_arm_dynamics(t, x, params), tspan, x0, options);
    
    % Store results
    t_all{end+1} = t_segment;
    x_all{end+1} = x_segment;
    
    % Update the initial condition for the next segment
    x0 = x_segment(end, :);
    
    % Update the trajectory start conditions for the next segment
    params.q_start = x0(1:3)';
    p_start = get_forward_kinematics(x0(1:3)', params.a2, params.a3);
    params.pos_start = p_start;
    params.traj_start_time = t_start;
end

% Concatenate all segments
t = t_all{1};
x = x_all{1};
for i = 2:length(t_all)
    % Add the duration of the previous segment to the time vector
    t_segment = t_all{i};
    x_segment = x_all{i};
    t = [t; t(end) + t_segment(2:end)];
    x = [x; x_segment(2:end,:)];
end
t_end = t(end);

disp('Simulation Complete. Preparing Animation...');

%% 4. Data Interpolation (SMOOTHING STEP)
% Create a fixed frame-rate time vector (e.g., 30 FPS)
fps = 30;
t_anim = t_start : 1/fps : t_end; 

% Interpolate the raw ODE solution onto this fixed timeline
% This prevents jitter from variable time steps
x_anim = interp1(t, x, t_anim); 

%% 5. Pre-calculate End Effector Path for Trace
addpath('visualization');
end_effector_path = get_end_effector_path(t_anim, x_anim, params);


%% 5. Plot Static Results
figure('Name', 'Joint Angles');
plot(t, x(:, 1:3), 'LineWidth', 2);
legend('Waist (q1)', 'Shoulder (q2)', 'Elbow (q3)');
xlabel('Time (s)');
ylabel('Angle (rad)');
title('Joint Angles vs Time');
grid on;

%% 6. 3D Animation
animate_robot(t_anim, x_anim, end_effector_path, target_pos, params, t_end, fps);

%% 7. Energy Check
% Using RAW data (t, x) for physics accuracy
E_total = zeros(length(t), 1);
T_log = zeros(length(t), 1);
U_log = zeros(length(t), 1);

for i = 1:length(t)
    q_now = x(i, 1:3)';
    dq_now = x(i, 4:6)';
    
    % 1. Mass Matrix (B)
    B = get_B_matrix(q_now(1), q_now(2), q_now(3), ...
        params.m1, params.m2, params.m3, params.a2, params.a3, ...
        params.I1(1,1), params.I1(2,2), params.I1(3,3), ...
        params.I2(1,1), params.I2(2,2), params.I2(3,3), ...
        params.I3(1,1), params.I3(2,2), params.I3(3,3));
        
    % 2. Kinetic Energy
    T_val = 0.5 * dq_now' * B * dq_now;
    
    % 3. Potential Energy (Recalculating Transforms for CoM heights)
    % A1..A3 setup copied from kinematics logic
    c1=cos(q_now(1)); s1=sin(q_now(1));
    c2=cos(q_now(2)); s2=sin(q_now(2));
    c3=cos(q_now(3)); s3=sin(q_now(3));
    
    A1=[c1 0 s1 0; s1 0 -c1 0; 0 1 0 0; 0 0 0 1];
    A2=[c2 -s2 0 params.a2*c2; s2 c2 0 params.a2*s2; 0 0 1 0; 0 0 0 1];
    A3=[c3 -s3 0 params.a3*c3; s3 c3 0 params.a3*s3; 0 0 1 0; 0 0 0 1];
    
    T1 = A1; T2 = T1*A2; T3 = T2*A3;
    
    p1 = T1(1:3, 4);
    p2 = T2(1:3, 4);
    p3 = T3(1:3, 4);
    
    % CoM Heights
    h1 = p1(3);             
    h2 = (p1(3) + p2(3))/2; 
    h3 = (p2(3) + p3(3))/2;
    
    U_val = params.m1*params.g*h1 + params.m2*params.g*h2 + params.m3*params.g*h3;
        
    E_total(i) = T_val + U_val;
    T_log(i) = T_val;
    U_log(i) = U_val;
end

figure;
plot(t, E_total, 'k', 'LineWidth', 2); hold on;
plot(t, T_log, '--r');
plot(t, U_log, '--b');
legend('Total Energy', 'Kinetic (T)', 'Potential (U)');
title('Energy Conservation Check');
xlabel('Time (s)');
ylabel('Energy (J)');
grid on;