function [p_des, v_des, a_des] = get_cartesian_traj(t, t_start, duration, p_start, p_end)
% GET_CARTESIAN_TRAJ
%
% Generates a straight-line trajectory for the end-effector in Cartesian space
%
% INPUTS:
%   t           : Current time (s)
%   t_start     : Trajectory start time (s)
%   duration    : Trajectory duration (s)
%   p_start     : Initial end-effector position [3x1] (m)
%   p_end       : Target end-effector position [3x1] (m)
%
% OUTPUTS:
%   p_des       : Desired position [3x1] (m)
%   v_des       : Desired velocity [3x1] (m/s)
%   a_des       : Desired acceleration [3x1] (m/s^2)

    %% 0. Handle Time Limits
    if t <= t_start
        p_des = p_start;
        v_des = zeros(3,1);
        a_des = zeros(3,1);
        return;
    elseif t >= (t_start + duration)
        p_des = p_end;
        v_des = zeros(3,1);
        a_des = zeros(3,1);
        return;
    end

    %% 1. Normalize Time
    tau = (t - t_start) / duration;

    %% 2. Calculate Polynomial Basis Functions
    s      = 3*tau^2 - 2*tau^3;
    s_dot  = 6*tau   - 6*tau^2;
    s_ddot = 6       - 12*tau;

    %% 3. Scale to Trajectory
    delta_p = p_end - p_start;

    p_des = p_start + delta_p * s;
    v_des = delta_p * s_dot / duration;
    a_des = delta_p * s_ddot / duration^2;

end
