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
target_pos = {[0.5; 0.5; 0.5], [0.0; -0.6; 0.4], [-0.5; 0.5; 0.2]}; 
params.vel_target = [0; 0; 0]; % Target velocity is always zero.

% --- Trajectory Timing ---
params.traj_start_time = 0.0; 
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
params.CONTROL_SPACE = 'OPERATIONAL';

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

%% 5b. Calculate "Ghost Goal" Path (Commanded Position)
% This determines where the controller WANTS the robot to be at every frame.
% It handles both Trajectory ON/OFF and Joint/Operational spaces.

ghost_path = zeros(length(t_anim), 3);
current_time_offset = 0;

% Re-initialize start conditions exactly as simulation started
x_start_seg = [q1_0; q2_0; q3_0; 0; 0; 0];
pos_start_seg = get_forward_kinematics(x_start_seg(1:3), params.a2, params.a3);

% Loop through each target segment to build the continuous ghost path
for i = 1:length(target_pos)
    % Duration of this specific segment (derived from simulation data)
    t_seg_duration = t_all{i}(end);
    
    % Find animation frames belonging to this segment
    seg_indices = find(t_anim >= current_time_offset & t_anim < (current_time_offset + t_seg_duration));
    
    % Goals for this segment
    pos_target_seg = target_pos{i};
    % Calculate Joint Target (IK) for Joint Space control
    q_target_seg   = get_inverse_kinematics(pos_target_seg(1), pos_target_seg(2), pos_target_seg(3), params.a2, params.a3);
    
    for k = seg_indices
        % Local time within the segment (e.g., 0.0 to 4.0s)
        t_local = t_anim(k) - current_time_offset;
        
        if params.USE_TRAJECTORY
            % --- TRAJECTORY MODE ---
            if strcmp(params.CONTROL_SPACE, 'JOINT')
                % 1. Get Desired Joint Angles
                [q_des, ~, ~] = get_cubic_traj(t_local, 0, params.traj_duration, x_start_seg(1:3), q_target_seg);
                % 2. FK to find where that puts the end effector
                p_des = get_forward_kinematics(q_des, params.a2, params.a3);
                ghost_path(k, :) = p_des';
                
            elseif strcmp(params.CONTROL_SPACE, 'OPERATIONAL')
                % 1. Get Desired Cartesian Position directly
                [p_des, ~, ~] = get_cartesian_traj(t_local, 0, params.traj_duration, pos_start_seg, pos_target_seg);
                ghost_path(k, :) = p_des';
            end
        else
            % --- STEP INPUT MODE ---
            % The command jumps instantly to the target
            ghost_path(k, :) = pos_target_seg';
        end
    end
    
    % Update start conditions for the next segment loop
    current_time_offset = current_time_offset + t_seg_duration;
    x_start_seg = x_all{i}(end, :)';
    pos_start_seg = get_forward_kinematics(x_start_seg(1:3), params.a2, params.a3);
end

% Ensure the last frame holds the final target to avoid drop-off
if ~isempty(ghost_path)
    ghost_path(end, :) = target_pos{end}';
end

% --- UPDATE ANIMATION CALL ---
% Pass 'ghost_path' as a new argument

%% 5. Plot Static Results
figure('Name', 'Joint Angles');
plot(t, x(:, 1:3), 'LineWidth', 2);
legend('Waist (q1)', 'Shoulder (q2)', 'Elbow (q3)');
xlabel('Time (s)');
ylabel('Angle (rad)');
title('Joint Angles vs Time');
grid on;

%% 6. 3D Animation
animate_robot(t_anim, x_anim, end_effector_path, target_pos, params, t_end, fps, ghost_path);


%% 8. Plot Tracking Error (New Section)
figure('Name', 'Cartesian Tracking Error');
hold on; grid on;

% Initialize storage for concatenated error data
all_time = [];
all_error_norm = [];
all_error_xyz = [];

% Re-iterate through segments to reconstruct the reference signal
current_time_offset = 0;

% Re-initialize start conditions exactly as they were at the start of the sim
x_start_seg = [q1_0; q2_0; q3_0; 0; 0; 0];
pos_start_seg = get_forward_kinematics(x_start_seg(1:3), params.a2, params.a3);

for i = 1:length(t_all)
    % Extract simulation data for this segment
    t_seg = t_all{i}; % Time vector (0 to 5s)
    x_seg = x_all{i}; % State vector
    
    % Current target for this segment
    pos_target_seg = target_pos{i};
    
    % Storage for this segment
    err_seg_xyz = zeros(length(t_seg), 3);
    err_seg_norm = zeros(length(t_seg), 1);
    
    for k = 1:length(t_seg)
        t_curr = t_seg(k);
        q_curr = x_seg(k, 1:3)';
        
        % 1. Get Actual Position (FK)
        pos_actual = get_forward_kinematics(q_curr, params.a2, params.a3);
        
        % 2. Get Desired Position (Re-calculate Reference correctly)
        if params.USE_TRAJECTORY
            if strcmp(params.CONTROL_SPACE, 'JOINT')
                % A. Joint Space: Reconstruct the angle trajectory first
                % We need the target angles for this segment (q_final)
                % Note: We can re-compute q_final using the segment target
                q_target_seg = get_inverse_kinematics(pos_target_seg(1), ...
                                                      pos_target_seg(2), ...
                                                      pos_target_seg(3), ...
                                                      params.a2, params.a3);
                                                      
                [q_des, ~, ~] = get_cubic_traj(t_curr, 0, ...
                                             params.traj_duration, ...
                                             x_start_seg(1:3), ...
                                             q_target_seg);
                
                % Convert desired angles to Cartesian to compare with actual
                pos_des = get_forward_kinematics(q_des, params.a2, params.a3);
                
            elseif strcmp(params.CONTROL_SPACE, 'OPERATIONAL')
                % B. Operational Space: Reconstruct the straight line
                [pos_des, ~, ~] = get_cartesian_traj(t_curr, 0, ...
                                                    params.traj_duration, ...
                                                    pos_start_seg, ...
                                                    pos_target_seg);
            end
        else
            % Step Input (No trajectory)
            pos_des = pos_target_seg;
        end
        
        % 3. Calculate Error
        err_seg_xyz(k, :) = (pos_des - pos_actual)';
        err_seg_norm(k) = norm(pos_des - pos_actual);
    end
    
    % Append to global lists for plotting
    % Shift time by the offset so segments appear sequentially
    all_time = [all_time; t_seg + current_time_offset];
    all_error_xyz = [all_error_xyz; err_seg_xyz];
    all_error_norm = [all_error_norm; err_seg_norm];
    
    % Update offset and start conditions for next loop (just like in the main sim)
    current_time_offset = current_time_offset + t_seg(end);
    x_start_seg = x_seg(end, :)';
    pos_start_seg = get_forward_kinematics(x_start_seg(1:3), params.a2, params.a3);
end

% --- PLOT 1: Error Norm (Scalar distance from target) ---
subplot(2,1,1);
plot(all_time, all_error_norm, 'k', 'LineWidth', 1.5);
title('Position Error Norm ||x_{des} - x_{act}||');
xlabel('Time (s)');
ylabel('Error (m)');
ylim([0, 1.5]);
grid on;

% --- PLOT 2: Component Error (X, Y, Z) ---
subplot(2,1,2);
plot(all_time, all_error_xyz, 'LineWidth', 1.5);
legend('X Error', 'Y Error', 'Z Error');
title('Component Errors');
xlabel('Time (s)');
ylabel('Error (m)');
ylim([-1, 1]);
grid on;