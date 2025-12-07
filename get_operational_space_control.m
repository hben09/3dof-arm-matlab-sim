function tau = get_operational_space_control(q_curr, dq_curr, pos_target, vel_target, params, G)
% GET_OPERATIONAL_SPACE_CONTROL
% Implements Siciliano Eq 8.110 (PD Control in Operational Space)
% Acts like a virtual spring pulling the end-effector to pos_target.

    %% 1. Calculate Forward Kinematics (Current X,Y,Z)
    % You can reuse your A matrices logic or just implement the direct equations
    % Simplified forward kinematics for the end effector position:
    c1=cos(q_curr(1)); s1=sin(q_curr(1));
    c2=cos(q_curr(2)); s2=sin(q_curr(2));
    c23=cos(q_curr(2)+q_curr(3)); s23=sin(q_curr(2)+q_curr(3));
    
    % Position of end effector (See derive_arm_equations for source)
    x_curr = c1 * (params.a2*c2 + params.a3*c23);
    y_curr = s1 * (params.a2*c2 + params.a3*c23);
    z_curr = params.a2*s2 + params.a3*s23;
    
    p_curr = [x_curr; y_curr; z_curr];

    %% 2. Get Jacobian
    % Calculate J (6x3 matrix)
    J_full = get_Jacobian(q_curr(1), q_curr(2), q_curr(3), params.a2, params.a3);
    
    % We are only controlling Position (X,Y,Z), so take top 3 rows
    J_pos = J_full(1:3, :);

    %% 3. Calculate Operational Space Velocity
    % v = J * dq
    v_curr = J_pos * dq_curr;

    %% 4. Control Law (Virtual Spring-Damper)
    % F_des = Kp * (Pos_Error) + Kd * (Vel_Error)
    
    % Tunable Cartesian Gains (N/m and Ns/m)
    Kp_cart = diag([500, 500, 500]); 
    Kd_cart = diag([50, 50, 50]);

    pos_err = pos_target - p_curr;
    vel_err = vel_target - v_curr;

    % Cartesian Force required
    F_des = Kp_cart * pos_err + Kd_cart * vel_err;

    %% 5. Map Force to Torque
    % tau = J' * F + Gravity
    tau = J_pos' * F_des + G;

end