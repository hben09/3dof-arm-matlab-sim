function [q, dq, ddq] = get_cubic_traj(t, t_start, duration, q_start, q_end)
% GET_CUBIC_TRAJ 
% Generates a cubic polynomial trajectory between two points.
%
% INPUTS:
%   t         : Current simulation time
%   t_start   : Time when the motion should start
%   duration  : How long the motion should take
%   q_start   : Starting joint angles (vector)
%   q_end     : Target joint angles (vector)
%
% OUTPUTS:
%   q    : Desired Position at time t
%   dq   : Desired Velocity at time t
%   ddq  : Desired Acceleration at time t

    % 1. Handle Time Limits
    if t <= t_start
        % Before motion starts: hold initial position
        q = q_start;
        dq = zeros(size(q_start));
        ddq = zeros(size(q_start));
        return;
    elseif t >= (t_start + duration)
        % After motion ends: hold final position
        q = q_end;
        dq = zeros(size(q_end));
        ddq = zeros(size(q_end));
        return;
    end

    % 2. Normalize Time (tau goes from 0 to 1)
    % See Siciliano Ch 4.2.1
    tau = (t - t_start) / duration;

    % 3. Calculate Polynomial Coefficients (Normalized)
    % For constraints: q(0)=qi, q(1)=qf, dq(0)=0, dq(1)=0
    % The polynomial s(tau) = 3*tau^2 - 2*tau^3
    s     = 3*tau^2 - 2*tau^3;
    s_dot = 6*tau   - 6*tau^2;       % Derivative wrt tau
    s_ddot= 6       - 12*tau;        % 2nd Derivative wrt tau

    % 4. Scale to Real World Units
    delta_q = q_end - q_start;

    q   = q_start + delta_q * s;
    
    % Chain rule: dq/dt = (dq/dtau) * (dtau/dt) where dtau/dt = 1/duration
    dq  = delta_q * s_dot * (1 / duration);
    
    % Chain rule: ddq/dt^2 = ... * (1/duration)^2
    ddq = delta_q * s_ddot * (1 / duration^2);

end