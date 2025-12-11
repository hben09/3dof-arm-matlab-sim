function xdot = f_arm_dynamics(t, x, params)
% F_ARM_DYNAMICS
%
% Calculates the differential equations of motion for the 3-DOF robotic arm
%
% INPUTS:
%   t           : Current time (s)
%   x           : State vector [q1; q2; q3; dq1; dq2; dq3]
%   params      : Structure containing physical parameters and control settings
%
% OUTPUTS:
%   xdot        : Time derivative of state [dq1; dq2; dq3; ddq1; ddq2; ddq3]

    %% 0. Extract State Vector
    q1 = x(1);  q2 = x(2);  q3 = x(3);
    dq1 = x(4); dq2 = x(5); dq3 = x(6);

    q_curr  = [q1; q2; q3];
    dq_curr = [dq1; dq2; dq3];

    %% 1. Extract Physical Parameters
    a2 = params.a2;  a3 = params.a3;
    m1 = params.m1;  m2 = params.m2;  m3 = params.m3;
    g = params.g;

    Ixx1 = params.I1(1,1); Iyy1 = params.I1(2,2); Izz1 = params.I1(3,3);
    Ixx2 = params.I2(1,1); Iyy2 = params.I2(2,2); Izz2 = params.I2(3,3);
    Ixx3 = params.I3(1,1); Iyy3 = params.I3(2,2); Izz3 = params.I3(3,3);

    %% 2. Calculate Dynamics Matrices
    B = get_B_matrix(q1, q2, q3, m1, m2, m3, a2, a3, ...
        Ixx1, Iyy1, Izz1, Ixx2, Iyy2, Izz2, Ixx3, Iyy3, Izz3);

    C = get_C_matrix(q1, q2, q3, dq1, dq2, dq3, m1, m2, m3, a2, a3, ...
        Ixx1, Iyy1, Izz1, Ixx2, Iyy2, Izz2, Ixx3, Iyy3, Izz3);

    G = get_G_vector(q1, q2, q3, m1, m2, m3, a2, a3, g);

    %% 3. Determine Reference Trajectory
    if strcmp(params.CONTROL_SPACE, 'JOINT')
        if params.USE_TRAJECTORY
            [ref_pos, ref_vel, ref_acc] = get_cubic_traj(t, params.traj_start_time, ...
                params.traj_duration, params.q_start, params.q_target);
        else
            ref_pos = params.q_target;
            ref_vel = zeros(3,1);
            ref_acc = zeros(3,1);
        end

    elseif strcmp(params.CONTROL_SPACE, 'OPERATIONAL')
        if params.USE_TRAJECTORY
            [ref_pos, ref_vel, ref_acc] = get_cartesian_traj(t, params.traj_start_time, ...
                params.traj_duration, params.pos_start, params.pos_target);
        else
            ref_pos = params.pos_target;
            ref_vel = zeros(3,1);
            ref_acc = zeros(3,1);
        end
    end

    %% 4. Calculate Control Torques
    if strcmp(params.CONTROL_SPACE, 'JOINT')
        if params.USE_INVERSE_DYNAMICS
            tau = get_joint_space_control(q_curr, dq_curr, ref_pos, ref_vel, ...
                params, ref_acc, 'inverse_dynamics');
        else
            tau = get_joint_space_control(q_curr, dq_curr, ref_pos, ref_vel, ...
                params, G);
        end

    elseif strcmp(params.CONTROL_SPACE, 'OPERATIONAL')
        if params.USE_INVERSE_DYNAMICS
            tau = get_operational_space_control(q_curr, dq_curr, ref_pos, ref_vel, ...
                params, ref_acc, 'inverse_dynamics');
        else
            tau = get_operational_space_control(q_curr, dq_curr, ref_pos, ref_vel, ...
                params, G);
        end
    end

    %% 5. Apply Friction
    damping_coeff = 0.5;
    tau = tau - damping_coeff * dq_curr;

    %% 6. Compute Acceleration
    q_dd = B \ (tau - C * dq_curr - G);
    xdot = [dq_curr; q_dd];

end
