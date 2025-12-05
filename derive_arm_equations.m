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
I1 = diag([Ixx1, Iyy1, Izz1])
I2 = diag([Ixx2, Iyy2, Izz2])
I3 = diag([Ixx3, Iyy3, Izz3])


%%%%%%%%%% Define Kinematics (Geometry) %%%%%%%%%%