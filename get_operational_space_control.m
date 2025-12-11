function tau = get_operational_space_control(q_curr, dq_curr, pos_target, vel_target, params, G)
% GET_OPERATIONAL_SPACE_CONTROL Compute joint torques using operational space control
%
% Implements Siciliano Eq 8.110 (PD Control in Operational Space).
% Acts like a virtual spring pulling the end-effector to pos_target.
%
% Inputs:
%   q_curr      - Current joint angles [3x1] (rad)
%   dq_curr     - Current joint velocities [3x1] (rad/s)
%   pos_target  - Target end-effector position [3x1] (m)
%   vel_target  - Target end-effector velocity [3x1] (m/s)
%   params      - Structure containing:
%                   .a2 - Link 2 length (m)
%                   .a3 - Link 3 length (m)
%                   .kp - Proportional gain
%                   .kd - Derivative gain
%   G           - Gravity compensation torques [3x1] (Nm)
%
% Outputs:
%   tau         - Commanded joint torques [3x1] (Nm)

    %% 1. Calculate Forward Kinematics (Current X,Y,Z)
    p_curr = get_forward_kinematics(q_curr, params.a2, params.a3);

    %% 2. Get Jacobian
    J_full = get_Jacobian(q_curr(1), q_curr(2), q_curr(3), params.a2, params.a3);
    J_pos = J_full(1:3, :);  % Only control position (X,Y,Z), take top 3 rows

    %% 3. Calculate Operational Space Velocity
    v_curr = J_pos * dq_curr;  % v = J * dq

    %% 4. Calculate Control Law
    Kp_cart = params.kp * eye(3);
    Kd_cart = params.kd * eye(3);

    pos_err = pos_target - p_curr;
    vel_err = vel_target - v_curr;

    %% 5. Compute Joint Torques
    F_des = Kp_cart * pos_err + Kd_cart * vel_err;
    tau = J_pos' * F_des + G;

end