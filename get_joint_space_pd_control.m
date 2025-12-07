function tau = get_joint_space_pd_control(q_curr, dq_curr, q_des, dq_des, G, params)
% GET_JOINT_SPACE_PD_CONTROL
%
% INPUTS:
%   q_curr, dq_curr   : Current state
%   q_des, dq_des     : Desired trajectory state (pos, vel)
%   G                 : Gravity vector
%   params            : Struct with gains

    %% 1. Extract Gains
    Kp = params.kp * eye(3); 
    Kd = params.kd * eye(3); 

    %% 2. Calculate Error
    e  = q_des - q_curr;
    de = dq_des - dq_curr;
    
    %% 3. PD Control Law + Gravity Compensation
    tau = Kp*e + Kd*de + G;
    
end