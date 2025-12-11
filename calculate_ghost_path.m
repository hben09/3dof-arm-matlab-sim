function ghost_path = calculate_ghost_path(t_anim, target_pos, t_all, x_all, params, q1_0, q2_0, q3_0)
% CALCULATE_GHOST_PATH Compute the desired trajectory path for visualization

%% 1. Initialize
ghost_path = zeros(length(t_anim), 3);
current_time_offset = 0;

x_start_seg = [q1_0; q2_0; q3_0; 0; 0; 0];
pos_start_seg = get_forward_kinematics(x_start_seg(1:3), params.a2, params.a3);

%% 2. Compute Desired Path for Each Segment
for i = 1:length(target_pos)
    t_seg_duration = t_all{i}(end);
    seg_indices = find(t_anim >= current_time_offset & t_anim < (current_time_offset + t_seg_duration));

    pos_target_seg = target_pos{i};
    q_target_seg = get_inverse_kinematics(pos_target_seg(1), pos_target_seg(2), pos_target_seg(3), params.a2, params.a3);

    for k = seg_indices
        t_local = t_anim(k) - current_time_offset;

        if params.USE_TRAJECTORY
            if strcmp(params.CONTROL_SPACE, 'JOINT')
                [q_des, ~, ~] = get_cubic_traj(t_local, 0, params.traj_duration, x_start_seg(1:3), q_target_seg);
                p_des = get_forward_kinematics(q_des, params.a2, params.a3);
                ghost_path(k, :) = p_des';
            elseif strcmp(params.CONTROL_SPACE, 'OPERATIONAL')
                [p_des, ~, ~] = get_cartesian_traj(t_local, 0, params.traj_duration, pos_start_seg, pos_target_seg);
                ghost_path(k, :) = p_des';
            end
        else
            ghost_path(k, :) = pos_target_seg';
        end
    end

    current_time_offset = current_time_offset + t_seg_duration;
    x_start_seg = x_all{i}(end, :)';
    pos_start_seg = get_forward_kinematics(x_start_seg(1:3), params.a2, params.a3);
end

%% 3. Ensure Final Point Matches Target
if ~isempty(ghost_path)
    ghost_path(end, :) = target_pos{end}';
end

end
