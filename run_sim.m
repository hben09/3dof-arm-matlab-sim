% Benjamin Hsu
% ME500 Final Project
%
% SCRIPT: run_sim.m
% PURPOSE: Run the dynamic simulation of the 3-DOF Robotic Arm

clear; clc; close all;

%% 1. Define Robot Parameters
params.m1 = 1.0;  % Mass of Waist (kg)
params.m2 = 5.0;  % Mass of Upper Arm (kg)
params.m3 = 3.0;  % Mass of Forearm (kg)

params.a2 = 0.5;  % Length of Upper Arm (m)
params.a3 = 0.5;  % Length of Forearm (m)

params.g  = 9.81; % Gravity (m/s^2)

% Inertia Matrices (Simplified as diagonal for testing)
% Modeled roughly as slender rods: I = 1/12 * m * L^2
I_rod2 = (1/12) * params.m2 * params.a2^2;
I_rod3 = (1/12) * params.m3 * params.a3^2;

params.I1 = diag([0.01, 0.01, 0.01]);      % Small inertia for waist
params.I2 = diag([0.01, I_rod2, I_rod2]);  % Upper arm
params.I3 = diag([0.01, I_rod3, I_rod3]);  % Forearm

%% 2. Define Simulation Settings
% t_end is simulation length in seconds.
t_start = 0;
t_end   = 15;
tspan   = [t_start, t_end];

% Initial Conditions
% State Vector x = [q1; q2; q3; dq1; dq2; dq3]
% Arm straight out horizontally
q1_0 = 0;          % Facing forward
q2_0 = 0;          % Shoulder horizontal
q3_0 = 0;          % Elbow straight

x0 = [q1_0; q2_0; q3_0; 0; 0; 0]; % Zero initial velocity

target_pos = [0.5; 0.5; 0.5];
q_des = get_inverse_kinematics(target_pos(1), target_pos(2), target_pos(3), params.a2, params.a3);
params.q_target = q_des;
params.kp = 1500; % Stiffness gain
params.kd = 150;  % Damping gain

%% 3. Run Simulation (ODE Solver)
disp('Running Simulation...');

% Fix ode45 options
options = odeset('RelTol', 1e-3, 'AbsTol', 1e-3);

% Pass 'options' as the last argument
[t, x] = ode15s(@(t,x) f_arm_dynamics(t, x, params), tspan, x0, options);

disp('Simulation Complete.');

%% 4. Plot Results
figure('Name', 'Joint Angles');
plot(t, x(:, 1:3), 'LineWidth', 2);
legend('Waist (q1)', 'Shoulder (q2)', 'Elbow (q3)');
xlabel('Time (s)');
ylabel('Angle (rad)');
title('Joint Angles vs Time (Free Fall)');
grid on;

%% 5. 3D Animation with Rectangles
figure('Name', '3D Animation');
axis_limit = params.a2 + params.a3 + 0.1;
axis([-axis_limit axis_limit -axis_limit axis_limit -axis_limit axis_limit]);
grid on; hold on;
xlabel('X'); ylabel('Y'); zlabel('Z');
view(135, 30); % Better camera angle
axis equal;    % Keeps the boxes from looking squashed

% Visual settings
link_width = 0.05; % Thickness of the arm
color_base = [0.2 0.2 0.2]; % Dark Grey
color_arm2 = [0   0   1];   % Blue
color_arm3 = [1   0   0];   % Red

for i = 1:5:length(t)
    cla; % Clear the previous frame
    
    % Get current angles
    q1_val = x(i,1);
    q2_val = x(i,2);
    q3_val = x(i,3);
    
    % NUMERICAL KINEMATICS

    % A1 (Base to Shoulder)
    A1 = [cos(q1_val) 0 sin(q1_val) 0;
          sin(q1_val) 0 -cos(q1_val) 0;
          0 1 0 0;
          0 0 0 1];
      
    % A2 (Shoulder to Elbow)
    A2 = [cos(q2_val) -sin(q2_val) 0 params.a2*cos(q2_val);
          sin(q2_val)  cos(q2_val) 0 params.a2*sin(q2_val);
          0 0 1 0;
          0 0 0 1];
      
    % A3 (Elbow to Wrist)
    A3 = [cos(q3_val) -sin(q3_val) 0 params.a3*cos(q3_val);
          sin(q3_val)  cos(q3_val) 0 params.a3*sin(q3_val);
          0 0 1 0;
          0 0 0 1];
          
    % Calculate Global Transforms
    T1 = A1;         % Shoulder Frame
    T2 = T1 * A2;    % Elbow Frame
    T3 = T2 * A3;    % Wrist Frame
    
    % DRAWING
    
    % Draw Base/Waist (Just a small box at T1 to show the pivot)
    draw_link_3d(T1, 0.1, link_width*1.5, color_base);
    
    % Draw Upper Arm (Link 2)
    % It ends at T2 and has length params.a2
    draw_link_3d(T2, params.a2, link_width, color_arm2);
    
    % Draw Forearm (Link 3)
    % It ends at T3 and has length params.a3
    draw_link_3d(T3, params.a3, link_width, color_arm3);
    
    % Set limits and title
    title(['Time: ' num2str(t(i), '%.2f') ' s']);
    axis([-axis_limit axis_limit -axis_limit axis_limit -axis_limit axis_limit]);
    
    drawnow;
    
    % Real-time pacing
    % pause(0.05);

end

%% 6. Energy Check
% Total Energy = Kinetic (T) + Potential (U)

E_total = zeros(length(t), 1);
T_log = zeros(length(t), 1);
U_log = zeros(length(t), 1);

for i = 1:length(t)
    % Unpack state
    q_now = x(i, 1:3)';
    dq_now = x(i, 4:6)';
    q1_val = q_now(1);
    q2_val = q_now(2);
    q3_val = q_now(3);
    
    % --- 1. Calculate Mass Matrix (B) ---
    B = get_B_matrix(q1_val, q2_val, q3_val, ...
        params.m1, params.m2, params.m3, params.a2, params.a3, ...
        params.I1(1,1), params.I1(2,2), params.I1(3,3), ...
        params.I2(1,1), params.I2(2,2), params.I2(3,3), ...
        params.I3(1,1), params.I3(2,2), params.I3(3,3));
        
    % --- 2. Calculate Kinetic Energy (T) ---
    T = 0.5 * dq_now' * B * dq_now;
    
    % --- 3. Calculate Potential Energy (U) ---
    % We need the height (Z) of the Center of Mass for each link
    % Re-using the kinematics logic:
    
    % A1 (Base -> Shoulder)
    A1 = [cos(q1_val) 0 sin(q1_val) 0;
          sin(q1_val) 0 -cos(q1_val) 0;
          0 1 0 0;
          0 0 0 1];
    % A2 (Shoulder -> Elbow)
    A2 = [cos(q2_val) -sin(q2_val) 0 params.a2*cos(q2_val);
          sin(q2_val)  cos(q2_val) 0 params.a2*sin(q2_val);
          0 0 1 0;
          0 0 0 1];
    % A3 (Elbow -> Wrist)
    A3 = [cos(q3_val) -sin(q3_val) 0 params.a3*cos(q3_val);
          sin(q3_val)  cos(q3_val) 0 params.a3*sin(q3_val);
          0 0 1 0;
          0 0 0 1];
          
    % Transform Matrices
    T1 = A1;
    T2 = T1 * A2;
    T3 = T2 * A3;
    
    % Extract Positions (4th column, Z is 3rd element)
    p1 = T1(1:3, 4);
    p2 = T2(1:3, 4);
    p3 = T3(1:3, 4);
    
    % Centers of Mass (Midpoints)
    p_com1 = p1;              % Link 1 (Waist) - approx
    p_com2 = (p1 + p2) / 2;   % Link 2 (Upper Arm)
    p_com3 = (p2 + p3) / 2;   % Link 3 (Forearm)
    
    % U = m * g * h
    U = params.m1 * params.g * p_com1(3) + ...
        params.m2 * params.g * p_com2(3) + ...
        params.m3 * params.g * p_com3(3);
        
    % --- 4. Store Total ---
    E_total(i) = T + U;
    T_log(i) = T;
    U_log(i) = U;
end

figure;
plot(t, E_total, 'LineWidth', 2);
hold on;
plot(t, T_log, '--r');
plot(t, U_log, '--b');
legend('Total Energy', 'Kinetic (T)', 'Potential (U)');
title('Energy Conservation Check');
xlabel('Time (s)');
ylabel('Energy (J)');
grid on;