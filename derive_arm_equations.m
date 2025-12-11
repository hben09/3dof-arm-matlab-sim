% DERIVE_ARM_EQUATIONS
%
% Symbolically derives the equations of motion for a 3-DOF robotic arm
% Generates MATLAB functions for dynamics matrices (B, C, G) and kinematics (J, J_dot)

clear; clc;

%% 0. Define Symbolic Variables
syms a2 a3 m1 m2 m3 g real
syms Ixx1 Iyy1 Izz1 Ixx2 Iyy2 Izz2 Ixx3 Iyy3 Izz3 real
syms q1 q2 q3 dq1 dq2 dq3 real

q = [q1; q2; q3];
dq = [dq1; dq2; dq3];

I1 = diag([Ixx1, Iyy1, Izz1]);
I2 = diag([Ixx2, Iyy2, Izz2]);
I3 = diag([Ixx3, Iyy3, Izz3]);

%% 1. Define DH Parameters
DH = [   0       pi/2      0      q1;
         a2      0         0      q2;
         a3      0         0      q3  ];

%% 2. Generate Transformation Matrices
for i = 1:3
    a_i     = DH(i, 1);
    alpha_i = DH(i, 2);
    d_i     = DH(i, 3);
    theta_i = DH(i, 4);

    A{i} = [cos(theta_i)  -sin(theta_i)*cos(alpha_i)   sin(theta_i)*sin(alpha_i)   a_i*cos(theta_i);
            sin(theta_i)   cos(theta_i)*cos(alpha_i)  -cos(theta_i)*sin(alpha_i)   a_i*sin(theta_i);
            0              sin(alpha_i)                cos(alpha_i)                d_i;
            0              0                           0                           1];
end

A1 = A{1};
A2 = A{2};
A3 = A{3};

%% 3. Forward Kinematics
T01 = A1;
T02 = A1*A2;
T03 = A1*A2*A3;

R1 = T01(1:3, 1:3);
R2 = T02(1:3, 1:3);
R3 = T03(1:3, 1:3);

p1 = T01(1:3, 4);
p2 = T02(1:3, 4);
p3 = T03(1:3, 4);

%% 4. Define Centers of Mass
p_com1 = p1;
p_com2 = (p1 + p2) / 2;
p_com3 = (p2 + p3) / 2;

%% 5. Calculate Jacobians
JP1 = jacobian(p_com1, q);
JP2 = jacobian(p_com2, q);
JP3 = jacobian(p_com3, q);

z0 = [0; 0; 1];
z1 = R1 * [0; 0; 1];
z2 = R2 * [0; 0; 1];

JO1 = [z0,       [0;0;0], [0;0;0]];
JO2 = [z0,       z1,      [0;0;0]];
JO3 = [z0,       z1,      z2];

J_end_effector = [JP3; JO3];

J_pos = JP3;
J_dot = diff(J_pos, q1)*dq1 + diff(J_pos, q2)*dq2 + diff(J_pos, q3)*dq3;

%% 6. Calculate Mass Matrix (B)
B = (m1 * JP1.' * JP1 + JO1.' * R1 * I1 * R1.' * JO1) + ...
    (m2 * JP2.' * JP2 + JO2.' * R2 * I2 * R2.' * JO2) + ...
    (m3 * JP3.' * JP3 + JO3.' * R3 * I3 * R3.' * JO3);
B = simplify(B);

%% 7. Calculate Gravity Vector (G)
g_vec_world = [0; 0; -g];

U = - (m1 * g_vec_world.' * p_com1) - ...
    (m2 * g_vec_world.' * p_com2) - ...
    (m3 * g_vec_world.' * p_com3);

G_vect = jacobian(U, q).';
G_vect = simplify(G_vect);

%% 8. Calculate Coriolis Matrix (C)
n = 3;
C_mat = sym(zeros(n,n));

for k = 1:n
    for j = 1:n
        c_kj = 0;
        for i = 1:n
            db_ij_dqk = diff(B(i,j), q(k));
            db_ik_dqj = diff(B(i,k), q(j));
            db_jk_dqi = diff(B(j,k), q(i));

            c_ijk = 0.5 * (db_ij_dqk + db_ik_dqj - db_jk_dqi);

            c_kj = c_kj + c_ijk * dq(i);
        end
        C_mat(k,j) = c_kj;
    end
end
C_mat = simplify(C_mat);

%% 9. Export to MATLAB Functions
disp('Generating MATLAB function files...');

output_dir = 'generated_functions';
if ~exist(output_dir, 'dir')
   mkdir(output_dir);
end
addpath(output_dir);

q_vars = {q1, q2, q3};
dq_vars = {dq1, dq2, dq3};
mass_vars = {m1, m2, m3};
geom_vars = {a2, a3};
inertia_vars = {Ixx1, Iyy1, Izz1, Ixx2, Iyy2, Izz2, Ixx3, Iyy3, Izz3};

matlabFunction(B, 'File', fullfile(output_dir, 'get_B_matrix'), ...
    'Vars', [q_vars, mass_vars, geom_vars, inertia_vars]);
matlabFunction(C_mat, 'File', fullfile(output_dir, 'get_C_matrix'), ...
    'Vars', [q_vars, dq_vars, mass_vars, geom_vars, inertia_vars]);
matlabFunction(G_vect, 'File', fullfile(output_dir, 'get_G_vector'), ...
    'Vars', [q_vars, mass_vars, geom_vars, {g}]);
matlabFunction(J_end_effector, 'File', fullfile(output_dir, 'get_Jacobian'), ...
    'Vars', [q_vars, geom_vars]);
matlabFunction(J_dot, 'File', fullfile(output_dir, 'get_J_dot'), ...
    'Vars', [q_vars, dq_vars, geom_vars]);

disp(['SUCCESS! Files generated in "' output_dir '" folder.']);
