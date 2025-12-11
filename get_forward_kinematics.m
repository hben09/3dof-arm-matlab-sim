function p = get_forward_kinematics(q, a2, a3)
% GET_FORWARD_KINEMATICS
%
% Calculates end-effector position from joint angles
%
% INPUTS:
%   q           : Joint angles [3x1] (rad)
%   a2, a3      : Link lengths (m)
%
% OUTPUTS:
%   p           : End-effector position [3x1] (m)

    %% 0. Extract Joint Angles
    q1 = q(1);
    q2 = q(2);
    q3 = q(3);

    %% 1. Calculate Trigonometric Terms
    c1 = cos(q1); s1 = sin(q1);
    c2 = cos(q2); s2 = sin(q2);
    c23 = cos(q2 + q3); s23 = sin(q2 + q3);

    %% 2. Compute End-Effector Position
    x = c1 * (a2*c2 + a3*c23);
    y = s1 * (a2*c2 + a3*c23);
    z = a2*s2 + a3*s23;

    p = [x; y; z];

end
