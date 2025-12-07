function tau = get_control_torque(q_curr, dq_curr, q_des, dq_des, ddq_des, ...
                                  B, C, G, params)
% GET_CONTROL_TORQUE (Computed Torque / Inverse Dynamics)
% Implementation of Siciliano Eq (8.57) and (8.60)
%
% INPUTS:
%   q_curr, dq_curr   : Current state
%   q_des, dq_des, ddq_des : Desired trajectory state (pos, vel, acc)
%   B, C, G           : Dynamics matrices (Mass, Coriolis, Gravity)
%   params            : Struct with gains and friction info

    %% 1. Extract Gains
    % Computed torque allows us to tune Kp and Kd based on natural frequency (wn)
    % Kp = wn^2, Kd = 2*zeta*wn (See Siciliano Sec 8.5.2)
    Kp = params.kp * eye(3); 
    Kd = params.kd * eye(3); 

    %% 2. Calculate Error
    e  = q_des - q_curr;
    de = dq_des - dq_curr;
    
    %% 3. Inverse Dynamics Control Law
    % Calculate the linear control input 'y' (Siciliano Eq 8.60)
    % y represents the desired acceleration corrected by error feedback
    y = ddq_des + Kd*de + Kp*e;
    
    % Friction Compensation (if you want to cancel the physical damping)
    % Note: In your f_arm_dynamics, you apply damping as: tau - damping*dq
    % To cancel it, we ADD it here.
    damping_coeff = 0.5; 
    F_comp = damping_coeff * dq_curr;

    % Calculate final torque (Siciliano Eq 8.57)
    % tau = B*y + Nonlinear_Terms
    % Nonlinear_Terms = C*dq + F*dq + G
    n = C * dq_curr + F_comp + G;
    
    tau = B * y + n;
    
end