% Benjamin Hsu
% ME500 Final Project
%
% SCRIPT: derive_arm_equations.m
% PURPOSE: Symobolically derive the equations of motion for a 3-DOF arm.

clear; clc;

%% Define Variables

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


%% Define Kinematics (Geometry)

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

%% Calculate Jacobians

% Linear Velocity Jacobians for CoM
JP1 = jacobian(p_com1, q);
JP2 = jacobian(p_com2, q);
JP3 = jacobian(p_com3, q);

% Angular Velocity Jacobians 
z0 = [0; 0; 1];          % Base Z
z1 = R1 * [0; 0; 1];     % Link 1 Z
z2 = R2 * [0; 0; 1];     % Link 2 Z

JO1 = [z0,       [0;0;0], [0;0;0]];
JO2 = [z0,       z1,      [0;0;0]];
JO3 = [z0,       z1,      z2];

%% Calculate Dynamics (Equations of Motion)

%%% Mass Matrix (B) %%%
% B = m * J_P'*J_P  +  J_O'*R*I*R'*J_O

B = (m1 * JP1.' * JP1 + JO1.' * R1 * I1 * R1.' * JO1) + (m2 * JP2.' * JP2 + JO2.' * R2 * I2 * R2.' * JO2) + (m3 * JP3.' * JP3 + JO3.' * R3 * I3 * R3.' * JO3);
B = simplify(B);

%%% Gravity Vector (g_vect) %%%
g_vec_world = [0; 0; -g];

U = - (m1 * g_vec_world.' * p_com1) - (m2 * g_vec_world.' * p_com2) - (m3 * g_vec_world.' * p_com3);

% g(q) is the partial derivative of U with respect to q
G_vect = jacobian(U, q).';
G_vect = simplify(G_vect);

%%% Coriolis (C) %%%
n = 3;
C_mat = sym(zeros(n,n));

for k = 1:n
    for j = 1:n
        c_kj = 0;
        for i = 1:n
            % 1. Calculate the partial derivatives of B
            % diff(function, variable)
            db_ij_dqk = diff(B(i,j), q(k));
            db_ik_dqj = diff(B(i,k), q(j));
            db_jk_dqi = diff(B(j,k), q(i));
            
            % 2. Christoffel Formula
            c_ijk = 0.5 * (db_ij_dqk + db_ik_dqj - db_jk_dqi);
            
            % 3. Sum it up multiplied by velocity
            c_kj = c_kj + c_ijk * dq(i);
        end
        C_mat(k,j) = c_kj;
    end
end
C_mat = simplify(C_mat);

%% Export
% This saves the results to .m files so you can use them in the simulation

disp('Generating MATLAB function files...');

output_dir = 'generated_functions';
if ~exist(output_dir, 'dir')
   mkdir(output_dir);
end
addpath(output_dir);


matlabFunction(B, 'File', fullfile(output_dir, 'get_B_matrix'), ...
    'Vars', {q1, q2, q3, m1, m2, m3, a2, a3, Ixx1, Iyy1, Izz1, Ixx2, Iyy2, Izz2, Ixx3, Iyy3, Izz3});

matlabFunction(C_mat, 'File', fullfile(output_dir, 'get_C_matrix'), ...
    'Vars', {q1, q2, q3, dq1, dq2, dq3, m1, m2, m3, a2, a3, Ixx1, Iyy1, Izz1, Ixx2, Iyy2, Izz2, Ixx3, Iyy3, Izz3});

matlabFunction(G_vect, 'File', fullfile(output_dir, 'get_G_vector'), ...
    'Vars', {q1, q2, q3, m1, m2, m3, a2, a3, g});

% The Geometric Jacobian J = [JP; JO]
% We only need the top 3 rows (Linear Velocity) if we only control Position (X,Y,Z)
J_linear = [JP1; JP2; JP3]; % Actually, usually we only care about the END EFFECTOR Jacobian (JP3)

% Let's export the Jacobian for the End Effector (Link 3)
J_end_effector = [JP3; JO3]; 

matlabFunction(J_end_effector, 'File', fullfile(output_dir, 'get_Jacobian'), ...
    'Vars', {q1, q2, q3, a2, a3});

disp(['SUCCESS! Files generated in "' output_dir '" folder and added to path.']);