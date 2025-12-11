function plot_tracking_error(t_all, x_all, target_pos, params, q1_0, q2_0, q3_0)
% PLOT_TRACKING_ERROR Calculate and plot tracking error for the robot

%% 1. Initialize
all_time = [];
all_error_norm = [];
all_error_xyz = [];

current_time_offset = 0;
x_start_seg = [q1_0; q2_0; q3_0; 0; 0; 0];
pos_start_seg = get_forward_kinematics(x_start_seg(1:3), params.a2, params.a3);

%% 2. Calculate Tracking Error for Each Segment
for i = 1:length(t_all)
    t_seg = t_all{i};
    x_seg = x_all{i};

    pos_target_seg = target_pos{i};

    err_seg_xyz = zeros(length(t_seg), 3);
    err_seg_norm = zeros(length(t_seg), 1);

    for k = 1:length(t_seg)
        t_curr = t_seg(k);
        q_curr = x_seg(k, 1:3)';

        pos_actual = get_forward_kinematics(q_curr, params.a2, params.a3);

        if params.USE_TRAJECTORY
            if strcmp(params.CONTROL_SPACE, 'JOINT')
                q_target_seg = get_inverse_kinematics(pos_target_seg(1), pos_target_seg(2), pos_target_seg(3), params.a2, params.a3);
                [q_des, ~, ~] = get_cubic_traj(t_curr, 0, params.traj_duration, x_start_seg(1:3), q_target_seg);
                pos_des = get_forward_kinematics(q_des, params.a2, params.a3);
            elseif strcmp(params.CONTROL_SPACE, 'OPERATIONAL')
                [pos_des, ~, ~] = get_cartesian_traj(t_curr, 0, params.traj_duration, pos_start_seg, pos_target_seg);
            end
        else
            pos_des = pos_target_seg;
        end

        err_seg_xyz(k, :) = (pos_des - pos_actual)';
        err_seg_norm(k) = norm(pos_des - pos_actual);
    end

    all_time = [all_time; t_seg + current_time_offset];
    all_error_xyz = [all_error_xyz; err_seg_xyz];
    all_error_norm = [all_error_norm; err_seg_norm];

    current_time_offset = current_time_offset + t_seg(end);
    x_start_seg = x_seg(end, :)';
    pos_start_seg = get_forward_kinematics(x_start_seg(1:3), params.a2, params.a3);
end

%% 3. Plot Tracking Error
figure('Name', 'Cartesian Tracking Error');

subplot(2,1,1);
plot(all_time, all_error_norm, 'k', 'LineWidth', 1.5);
title('Position Error Norm ||x_{des} - x_{act}||');
xlabel('Time (s)');
ylabel('Error (m)');
ylim([0, 1.5]);
grid on;

subplot(2,1,2);
plot(all_time, all_error_xyz, 'LineWidth', 1.5);
legend('X Error', 'Y Error', 'Z Error');
title('Component Errors');
xlabel('Time (s)');
ylabel('Error (m)');
ylim([-1, 1]);
grid on;

end
