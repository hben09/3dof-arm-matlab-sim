% plot_robot_setup.m
% Generates a "Problem Setup" diagram for the 3-DOF Robot Arm Report
% clearly showing links, joints, and the workspace.

clear; clc; close all;

%% 1. Define Robot Parameters (Matches Simulation Setup)
a2 = 0.5; % Length of Link 2 (Shoulder to Elbow) [m]
a3 = 0.5; % Length of Link 3 (Elbow to End-Effector) [m]

% Define a "Neutral" Pose for visualization (not singular, all links visible)
q = [pi/4;      % q1: Waist rotation (45 deg)
     pi/6;      % q2: Shoulder elevation (30 deg)
     -pi/3];    % q3: Elbow flexion (-60 deg)

%% 2. Forward Kinematics (Calculate Joint Positions)
% Base Frame (0)
P0 = [0; 0; 0];

% Waist Frame (1) - Rotates about Z0
% (Technically at the same position as base for this model, but purely rotational)
P1 = P0; 

% Shoulder Frame (2) - Rotates about Z1
% Located at the same physical point as waist but after rotation
P2 = P1;

% Elbow Frame (3) - Rotates about Z2
% Position depends on q1 and q2
x_elbow = cos(q(1)) * (a2 * cos(q(2)));
y_elbow = sin(q(1)) * (a2 * cos(q(2)));
z_elbow = a2 * sin(q(2));
P3 = [x_elbow; y_elbow; z_elbow];

% End-Effector Frame (ee)
% Position depends on q1, q2, and q3
c1 = cos(q(1)); s1 = sin(q(1));
c2 = cos(q(2)); s2 = sin(q(2));
s23 = sin(q(2) + q(3));
c23 = cos(q(2) + q(3));

x_ee = c1 * (a2*c2 + a3*c23);
y_ee = s1 * (a2*c2 + a3*c23);
z_ee = a2*s2 + a3*s23;
P_ee = [x_ee; y_ee; z_ee];

%% 3. Visualization
figure('Color', 'w', 'Name', '3-DOF Arm Problem Setup');
hold on; grid on; axis equal;
view(135, 30); % Set a nice isometric viewing angle

% Plot Links
lw = 8; % Line width for links
% Link 1 (Base/Waist is just a point/base here, usually drawn as vertical line if d1 exists)
plot3([0 0], [0 0], [-0.2 0], 'k-', 'LineWidth', 10); % Base stand

% Link 2 (Shoulder -> Elbow)
plot3([P2(1) P3(1)], [P2(2) P3(2)], [P2(3) P3(3)], 'b-', 'LineWidth', lw);

% Link 3 (Elbow -> End-Effector)
plot3([P3(1) P_ee(1)], [P3(2) P_ee(2)], [P3(3) P_ee(3)], 'r-', 'LineWidth', lw);

% Plot Joints (spheres)
plot3(P2(1), P2(2), P2(3), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k'); % Shoulder
plot3(P3(1), P3(2), P3(3), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');  % Elbow
plot3(P_ee(1), P_ee(2), P_ee(3), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g'); % End-Effector

% Plot Ground Plane for context
patch([-1 1 1 -1], [-1 -1 1 1], [-0.2 -0.2 -0.2 -0.2], [0.9 0.9 0.9], 'EdgeColor', 'none', 'FaceAlpha', 0.5);

%% 4. Add Coordinate Frames (Optional but Recommended)
scale = 0.2; % Length of axis arrows

% Base Frame (Frame 0)
quiver3(0,0,0, scale,0,0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5); % X0
quiver3(0,0,0, 0,scale,0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5); % Y0
quiver3(0,0,0, 0,0,scale, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5); % Z0
text(scale, 0, 0, 'X_0'); text(0, scale, 0, 'Y_0'); text(0, 0, scale, 'Z_0');

%% 5. Formatting
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('3-DOF Anthropomorphic Arm Configuration');
legend({'Base', 'Link 2 (Upper Arm)', 'Link 3 (Forearm)', 'Joints', 'End-Effector'}, 'Location', 'northeast');

% Set axis limits to look good
xlim([-0.8 0.8]); ylim([-0.8 0.8]); zlim([-0.2 0.8]);

% Make fonts larger for the report
set(gca, 'FontSize', 12);