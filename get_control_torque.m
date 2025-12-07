function tau = get_control_torque(q_curr, dq_curr, q_des, dq_des, params, G_comp)
% GET_CONTROL_TORQUE 
% Calculates joint torques
%
% INPUTS:
%   q_curr  : Current joint positions [3x1]
%   dq_curr : Current joint velocities [3x1]
%   params  : Struct containing targets (q_target) and gains (kp, kd)
%   G_comp  : Gravity vector for compensation [3x1]
%
% OUTPUT:
%   tau     : Control torque vector [3x1]

    % 1. Extract Gains
    Kp = params.kp * eye(3); 
    Kd = params.kd * eye(3); 

    % 2. Calculate Error
    e  = q_des - q_curr;
    de = dq_des - dq_curr; % Tracking error (target vel - current vel)
    
    % 3. Control Law
    % tau = Kp*e + Kd*de + Gravity
    tau = Kp*e + Kd*de + G_comp;
    
end