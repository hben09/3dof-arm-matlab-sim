function p = get_forward_kinematics(q, a2, a3)
% GET_FORWARD_KINEMATICS
% Calculates End-Effector Position (x,y,z) from Joint Angles (q)
% Matches the logic in derive_arm_equations.m

    q1 = q(1);
    q2 = q(2);
    q3 = q(3);
    
    c1 = cos(q1); s1 = sin(q1);
    c2 = cos(q2); s2 = sin(q2);
    c23 = cos(q2 + q3); s23 = sin(q2 + q3);
    
    % Position Equations (Anthropomorphic Arm)
    x = c1 * (a2*c2 + a3*c23);
    y = s1 * (a2*c2 + a3*c23);
    z = a2*s2 + a3*s23;
    
    p = [x; y; z];
end