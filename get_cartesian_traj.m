function [p_des, v_des] = get_cartesian_traj(t, t_start, duration, p_start, p_end)
% GET_CARTESIAN_TRAJ
% Generates a straight-line trajectory for the End-Effector (Operational Space)
% 
% INPUTS:
%   p_start : Starting coordinate [x;y;z]
%   p_end   : Target coordinate [x;y;z]
%
% OUTPUTS:
%   p_des   : Current desired position [3x1]
%   v_des   : Current desired linear velocity [3x1]

    if t <= t_start
        p_des = p_start;
        v_des = [0;0;0];
        return;
    elseif t >= (t_start + duration)
        p_des = p_end;
        v_des = [0;0;0];
        return;
    end

    % Normalized time (0 to 1)
    tau = (t - t_start) / duration;
    
    % Cubic Polynomial Scaling (Smooth accel/decel)
    s = 3*tau^2 - 2*tau^3;       % Position scale
    ds = (6*tau - 6*tau^2) / duration; % Velocity scale (Chain rule)
    
    % Interpolate Vector
    delta_p = p_end - p_start;
    
    % p(t) = p_start + (p_end - p_start) * s(t) 
    p_des = p_start + delta_p * s;
    
    % v(t) = (p_end - p_start) * s_dot(t) 
    v_des = delta_p * ds;
    
end