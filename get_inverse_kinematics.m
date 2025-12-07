function q_target = get_inverse_kinematics(px, py, pz, a2, a3)
% GET_INVERSE_KINEMATICS
% Calculates joint angles (q) needed to reach a cartesian position (px, py, pz)
% Based on Siciliano Section 2.12.4 (Pages 96-98)
%
% INPUTS:
%   px, py, pz : Target coordinates
%   a2, a3     : Link lengths
%
% OUTPUT:
%   q_target   : [theta1; theta2; theta3] (The "Elbow Down" solution)

    %% 1. Solve Joint 1 (Waist)
    % Simple trigonometry in the top-down view
    % Eq 2.109 (Page 98)
    theta1 = atan2(py, px);
    
    %% 2. Solve Joint 3 (Elbow)
    % Using Law of Cosines on the vertical plane triangle
    % We need the cosine of theta3 (c3)
    % r^2 = x^2 + y^2 + z^2
    r_sq = px^2 + py^2 + pz^2;
    
    % Eq 2.98 (Page 97)
    num = r_sq - a2^2 - a3^2;
    den = 2 * a2 * a3;
    c3 = num / den;
    
    % Safety Check: Is the target reachable?
    if abs(c3) > 1
        error('Target out of reach! c3 = %f', c3);
    end
    
    % Calculate Sine of theta3 (s3)
    % We choose Positive for "Elbow Down" solution (Siciliano 2.100)
    s3 = -sqrt(1 - c3^2); 
    
    theta3 = atan2(s3, c3);
    
    %% 3. Solve Joint 2 (Shoulder)
    % This is derived from the simultaneous equations for height and radius
    % Eq 2.103 and 2.104 (Page 97)
    
    % r_planar is the projected distance on the ground
    r_planar = sqrt(px^2 + py^2); 
    
    % We essentially solve the triangle formed by the arm in the vertical plane
    % theta2 = angle_to_wrist - angle_inside_triangle
    
    % Denominator (Determinant)
    D = a2^2 + a3^2 + 2*a2*a3*c3;
    
    % Calculate Cosine and Sine of Theta 2
    c2 = (r_planar * (a2 + a3*c3) + pz * (a3*s3)) / D;
    s2 = (pz * (a2 + a3*c3) - r_planar * (a3*s3)) / D;
    
    theta2 = atan2(s2, c2);
    
    %% 4. Package Output
    q_target = [theta1; theta2; theta3];

end