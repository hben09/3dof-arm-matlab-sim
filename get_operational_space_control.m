function tau = get_operational_space_control(q_curr, dq_curr, pos_target, vel_target, params, G)
% GET_OPERATIONAL_SPACE_CONTROL
% Implements Siciliano Eq 8.110 (PD Control in Operational Space)
% Acts like a virtual spring pulling the end-effector to pos_target.

    %% 1. Calculate Forward Kinematics (Current X,Y,Z)
    p_curr = get_forward_kinematics(q_curr, params.a2, params.a3);
    %% 2. Get Jacobian
    % Calculate J (6x3 matrix)
    J_full = get_Jacobian(q_curr(1), q_curr(2), q_curr(3), params.a2, params.a3);
    
    % We are only controlling Position (X,Y,Z), so take top 3 rows
    J_pos = J_full(1:3, :);

    %% 3. Calculate Operational Space Velocity
    % v = J * dq
    v_curr = J_pos * dq_curr;

    %% Control Law
    % Calculate operational space velocity
    v_curr = J_pos * dq_curr;
    
    Kp_cart = params.kp * eye(3);
    Kd_cart = params.kd * eye(3);

    pos_err = pos_target - p_curr;
    
    % FIX: Use the actual target velocity passed in
    vel_err = vel_target - v_curr; 

    F_des = Kp_cart * pos_err + Kd_cart * vel_err;
    tau = J_pos' * F_des + G;

end