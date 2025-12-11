function q_target = get_inverse_kinematics(px, py, pz, a2, a3)
% GET_INVERSE_KINEMATICS
%
% Calculates joint angles needed to reach a Cartesian position
% Returns the "Elbow Down" solution
%
% INPUTS:
%   px, py, pz  : Target end-effector position (m)
%   a2, a3      : Link lengths (m)
%
% OUTPUTS:
%   q_target    : Joint angles [3x1] (rad)

    %% 0. Calculate Radial Distance
    r_sq = px^2 + py^2 + pz^2;
    r_planar = sqrt(px^2 + py^2);

    %% 1. Solve Joint 1 (Waist)
    theta1 = atan2(py, px);

    %% 2. Solve Joint 3 (Elbow)
    c3 = (r_sq - a2^2 - a3^2) / (2 * a2 * a3);

    if abs(c3) > 1
        error('Target out of reach! c3 = %f', c3);
    end

    s3 = -sqrt(1 - c3^2);  % Negative for elbow down
    theta3 = atan2(s3, c3);

    %% 3. Solve Joint 2 (Shoulder)
    D = a2^2 + a3^2 + 2*a2*a3*c3;

    c2 = (r_planar * (a2 + a3*c3) + pz * (a3*s3)) / D;
    s2 = (pz * (a2 + a3*c3) - r_planar * (a3*s3)) / D;

    theta2 = atan2(s2, c2);

    %% 4. Package Output
    q_target = [theta1; theta2; theta3];

end
