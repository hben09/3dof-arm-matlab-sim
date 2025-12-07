function tau = get_op_space_inverse_dynamics(q_curr, dq_curr, p_target, v_target, a_target, params)
% GET_OP_SPACE_INVERSE_DYNAMICS
% Fully dynamic Operational Space Control (Computed Torque in Cartesian Space)
%
% INPUTS:
%   p_target, v_target, a_target : Desired Motion [3x1] each

    %% 1. Get Dynamics & Kinematics
    % We need ALL matrices now
    B = get_B_matrix(q_curr(1), q_curr(2), q_curr(3), ...
        params.m1, params.m2, params.m3, params.a2, params.a3, ...
        params.I1(1,1), params.I1(2,2), params.I1(3,3), ...
        params.I2(1,1), params.I2(2,2), params.I2(3,3), ...
        params.I3(1,1), params.I3(2,2), params.I3(3,3));
        
    C = get_C_matrix(q_curr(1), q_curr(2), q_curr(3), dq_curr(1), dq_curr(2), dq_curr(3), ...
        params.m1, params.m2, params.m3, params.a2, params.a3, ...
        params.I1(1,1), params.I1(2,2), params.I1(3,3), ...
        params.I2(1,1), params.I2(2,2), params.I2(3,3), ...
        params.I3(1,1), params.I3(2,2), params.I3(3,3));
        
    G = get_G_vector(q_curr(1), q_curr(2), q_curr(3), ...
        params.m1, params.m2, params.m3, params.a2, params.a3, params.g);

    J_full = get_Jacobian(q_curr(1), q_curr(2), q_curr(3), params.a2, params.a3);
    J_pos  = J_full(1:3, :); % Top 3 rows only (X,Y,Z)

    J_dot  = get_J_dot(q_curr(1), q_curr(2), q_curr(3), dq_curr(1), dq_curr(2), dq_curr(3), ...
                       params.a2, params.a3);

    %% 2. Calculate Current Cartesian State
    % We need current pos (p_curr) and current vel (v_curr)
    % (You can copy the FK logic or call get_forward_kinematics)
    p_curr = get_forward_kinematics(q_curr, params.a2, params.a3);
    v_curr = J_pos * dq_curr;

    %% 3. Cartesian Control Law (Outer Loop)
    % Determines the commanded ACCELERATION in Cartesian space
    Kp = params.kp * eye(3); 
    Kd = params.kd * eye(3);
    
    e = p_target - p_curr;
    de = v_target - v_curr;
    
    % y is the desired Cartesian acceleration
    y = a_target + Kd*de + Kp*e;

    %% 4. Inverse Dynamics (Inner Loop)
    % Map Cartesian acceleration 'y' to Joint Torque 'tau'
    % Formula: tau = B * J^-1 * (y - J_dot*dq) + C*dq + G
    
    % Note: Using matrix backslash (\) acts as inverse
    % This part essentially solves for the joint accelerations (ddq) 
    % that would produce the cartesian acceleration (y)
    
    inv_J_term = J_pos \ (y - J_dot * dq_curr);
    
    tau = B * inv_J_term + C * dq_curr + G;

end