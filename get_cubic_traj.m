function [q, dq, ddq] = get_cubic_traj(t, t_start, duration, q_start, q_end)
% GET_CUBIC_TRAJ
%
% Generates a cubic polynomial trajectory between two joint configurations
%
% INPUTS:
%   t           : Current time (s)
%   t_start     : Trajectory start time (s)
%   duration    : Trajectory duration (s)
%   q_start     : Initial joint angles [3x1] (rad)
%   q_end       : Target joint angles [3x1] (rad)
%
% OUTPUTS:
%   q           : Desired position [3x1] (rad)
%   dq          : Desired velocity [3x1] (rad/s)
%   ddq         : Desired acceleration [3x1] (rad/s^2)

    %% 0. Handle Time Limits
    if t <= t_start
        q = q_start;
        dq = zeros(size(q_start));
        ddq = zeros(size(q_start));
        return;
    elseif t >= (t_start + duration)
        q = q_end;
        dq = zeros(size(q_end));
        ddq = zeros(size(q_end));
        return;
    end

    %% 1. Normalize Time
    tau = (t - t_start) / duration;

    %% 2. Calculate Polynomial Basis Functions
    s      = 3*tau^2 - 2*tau^3;
    s_dot  = 6*tau   - 6*tau^2;
    s_ddot = 6       - 12*tau;

    %% 3. Scale to Trajectory
    delta_q = q_end - q_start;

    q   = q_start + delta_q * s;
    dq  = delta_q * s_dot / duration;
    ddq = delta_q * s_ddot / duration^2;

end
