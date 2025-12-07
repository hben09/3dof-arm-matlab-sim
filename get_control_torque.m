function tau = get_control_torque(q_curr, dq_curr, params, G_comp)
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

    %% 1. Extract Control Parameters
    q_des = params.q_target;
    dq_des = [0; 0; 0]; % Target velocity (0 for static setpoint)
    
    % Expand scalar gains to diagonal matrices
    Kp = params.kp * eye(3); 
    Kd = params.kd * eye(3); 

    %% 2. Calculate Error
    e  = q_des - q_curr;
    de = dq_des - dq_curr;
    
    %% 3. Control Law
    % Law: tau = Kp*error + Kd*error_dot + Gravity_Compensation
    tau = Kp*e + Kd*de + G_comp;
    
end