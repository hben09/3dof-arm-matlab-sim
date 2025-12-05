% Benjamin Hsu
% ME500 Final Project
%
% SCRIPT: derive_arm_equations.m
% PURPOSE: Symobolically derive the equations of motion for a 3-DOF arm.

clear; clc;

%%%%%%%%%% Define Variables %%%%%%%%%%

% Mass and length of links
syms a2 a3 m1 m2 m3 g real

% Inertia Matrices
syms Ixx1 Iyy1 Izz1 Ixx2 Iyy2 Izz2 Ixx3 Iyy3 Izz3 real

% State Variables (Angles and Velocity)
syms q1 q2 q3 dq1 dq2 dq3 real

% Group into vectors for easy calculation
q = [q1; q2; q3]; % Position Vector
dq = [dq1; dq2; dq3]; % Velocity Vector

% Define Interia Matricies relative to COM
I1 = diag([Ixx1, Iyy1, Izz1]);
I2 = diag([Ixx2, Iyy2, Izz2]);
I3 = diag([Ixx3, Iyy3, Izz3]);


%%%%%%%%%% Define Kinematics (Geometry) %%%%%%%%%%

% DH Matricies from Siciliano Table 2.4
%        a    |  alpha  |  d  |  theta
DH = [   0       pi/2      0      q1;    % Link 1 (Waist)
         a2      0         0      q2;    % Link 2 (Shoulder)
         a3      0         0      q3  ]; % Link 3 (Elbow)



% Generate Transformation Matrices for each link
for i = 1:3
    % Extract parameters for current link
    a_i     = DH(i, 1);
    alpha_i = DH(i, 2);
    d_i     = DH(i, 3);
    theta_i = DH(i, 4);
    
    % The DH Matrix Formula
    A{i} = [cos(theta_i)  -sin(theta_i)*cos(alpha_i)   sin(theta_i)*sin(alpha_i)   a_i*cos(theta_i);
            sin(theta_i)   cos(theta_i)*cos(alpha_i)  -cos(theta_i)*sin(alpha_i)   a_i*sin(theta_i);
            0              sin(alpha_i)                cos(alpha_i)                d_i;
            0              0                           0                           1];
end

% Assign to easy variables
A1 = A{1};
A2 = A{2};
A3 = A{3};

% Forward Kinematics
T01 = A1; % Base to Shoulder
T02 = A1*A2; % Base to Elbow
T03 = A1*A2*A3; % Base to End Effector

% Extract Rotation Matrices (R)
R1 = T01(1:3, 1:3);
R2 = T02(1:3, 1:3);
R3 = T03(1:3, 1:3);

% Extract Joint Positions (Origins)
p1 = T01(1:3, 4);
p2 = T02(1:3, 4);
p3 = T03(1:3, 4);

% Define Centers of Mass (CoM)
% We assume CoM is exactly in the middle of the link
p_com1 = p1;                % Link 1 CoM (Approximation)
p_com2 = (p1 + p2) / 2;     % Link 2 CoM
p_com3 = (p2 + p3) / 2;     % Link 3 CoM

%%%%%%%%%% Calculate Jacobians %%%%%%%%%%