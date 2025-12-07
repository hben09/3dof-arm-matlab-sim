function [p_des, v_des, a_des] = get_cartesian_traj(t, t_start, duration, p_start, p_end)
% GET_CARTESIAN_TRAJ
% Generates a straight-line trajectory for the End-Effector
% Now includes ACCELERATION output.

    if t <= t_start
        p_des = p_start;
        v_des = [0;0;0];
        a_des = [0;0;0]; % Zero acceleration before start
        return;
    elseif t >= (t_start + duration)
        p_des = p_end;
        v_des = [0;0;0];
        a_des = [0;0;0]; % Zero acceleration after end
        return;
    end

    % Normalized time (0 to 1)
    tau = (t - t_start) / duration;
    
    % Cubic Polynomial Scaling
    s = 3*tau^2 - 2*tau^3;       
    
    % First Derivative (velocity scale)
    ds = (6*tau - 6*tau^2) / duration; 
    
    % Second Derivative (acceleration scale)
    % Chain rule: d/dt(ds/dtau * 1/dur) = d2s/dtau2 * (1/dur)^2
    dds = (6 - 12*tau) / duration^2; 
    
    % Interpolate Vector
    delta_p = p_end - p_start;
    
    % Outputs
    p_des = p_start + delta_p * s;
    v_des = delta_p * ds;
    a_des = delta_p * dds; % New acceleration output
    
end