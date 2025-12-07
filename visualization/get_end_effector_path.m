function end_effector_path = get_end_effector_path(t_anim, x_anim, params)
    % GET_END_EFFECTOR_PATH
    % Pre-calculate End Effector Path for Trace
    
    disp('Calculating end effector path for animation trace...');
    end_effector_path = zeros(length(t_anim), 3);
    for i = 1:length(t_anim)
        q = x_anim(i, 1:3)';
        end_effector_path(i, :) = get_forward_kinematics(q, params.a2, params.a3)';
    end
end
