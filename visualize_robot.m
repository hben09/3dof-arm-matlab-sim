function visualize_robot(t_anim, x_anim, target_pos, params, t_end, fps, ghost_path)
% VISUALIZE_ROBOT
%
% Animates the 3-link robotic arm with end-effector trace
%
% INPUTS:
%   t_anim      : Time vector for animation (s)
%   x_anim      : State history [N x 6] (angles and velocities)
%   target_pos  : Cell array of target positions {[x;y;z], ...}
%   params      : Structure containing robot parameters (a2, a3)
%   t_end       : Total animation duration (s)
%   fps         : Animation frame rate (frames/s)
%   ghost_path  : Optional commanded position trace [N x 3] (m)

    %% 0. Calculate End-Effector Path
    disp('Calculating end effector path for animation trace...');
    end_effector_path = zeros(length(t_anim), 3);
    for i = 1:length(t_anim)
        q = x_anim(i, 1:3)';
        end_effector_path(i, :) = get_forward_kinematics(q, params.a2, params.a3)';
    end

    %% 1. Initialize Figure
    h_fig = figure('Name', '3D Animation');
    axis_limit = params.a2 + params.a3 + 0.1;
    axis([-axis_limit axis_limit -axis_limit axis_limit -axis_limit axis_limit]);
    grid on; hold on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    view(135, 30);
    axis equal;

    %% 2. Draw Target Positions
    for i = 1:length(target_pos)
        plot3(target_pos{i}(1), target_pos{i}(2), target_pos{i}(3), ...
              'go', 'MarkerSize', 10, 'MarkerFaceColor', '#00CC00');
    end

    %% 3. Define Link Styles
    link_width = 0.05;
    color_base = [0.2 0.2 0.2];
    color_arm2 = [0   0   1];
    color_arm3 = [1   0   0];

    %% 4. Animation Loop
    disp('Starting Animation...');
    tic;

    while ishandle(h_fig)
        t_current_real = toc;

        if t_current_real > t_end
            break;
        end

        idx = round(t_current_real * fps) + 1;
        if idx > length(t_anim)
            idx = length(t_anim);
        end

        % 4a. Update Robot Links
        q = x_anim(idx, 1:3);
        [T1, T2, T3] = get_transforms(q, params.a2, params.a3);

        cla;

        % Redraw targets
        for i = 1:length(target_pos)
            plot3(target_pos{i}(1), target_pos{i}(2), target_pos{i}(3), ...
                  'go', 'MarkerSize', 10, 'MarkerFaceColor', '#00CC00');
        end

        % 4b. Update Ghost Marker
        if exist('ghost_path', 'var') && ~isempty(ghost_path)
            plot3(ghost_path(idx, 1), ghost_path(idx, 2), ghost_path(idx, 3), ...
                  'cx', 'MarkerSize', 12, 'LineWidth', 2);
        end

        % 4c. Update Trace
        if idx > 1
            plot3(end_effector_path(1:idx, 1), end_effector_path(1:idx, 2), ...
                  end_effector_path(1:idx, 3), 'm-', 'LineWidth', 1.5);
        end

        % 4d. Draw Robot Links
        draw_link_3d(T1, 0.1, link_width*1.5, color_base);
        draw_link_3d(T2, params.a2, link_width, color_arm2);
        draw_link_3d(T3, params.a3, link_width, color_arm3);

        title(['Time: ' num2str(t_anim(idx), '%.2f') ' s / ' num2str(t_end) ' s']);
        axis([-axis_limit axis_limit -axis_limit axis_limit -axis_limit axis_limit]);

        drawnow limitrate;
    end

end


function [T1, T2, T3] = get_transforms(q, a2, a3)
% GET_TRANSFORMS
%
% Computes transformation matrices for each link

    q1 = q(1); q2 = q(2); q3 = q(3);

    A1 = [cos(q1) 0 sin(q1) 0; sin(q1) 0 -cos(q1) 0; 0 1 0 0; 0 0 0 1];
    A2 = [cos(q2) -sin(q2) 0 a2*cos(q2); sin(q2) cos(q2) 0 a2*sin(q2); 0 0 1 0; 0 0 0 1];
    A3 = [cos(q3) -sin(q3) 0 a3*cos(q3); sin(q3) cos(q3) 0 a3*sin(q3); 0 0 1 0; 0 0 0 1];

    T1 = A1;
    T2 = T1 * A2;
    T3 = T2 * A3;

end


function draw_link_3d(T, len, width, color)
% DRAW_LINK_3D
%
% Draws a 3D rectangular link using a transformation matrix

    half_w = width / 2;
    vertices_local = [
        -len, -half_w, -half_w; -len,  half_w, -half_w; -len,  half_w,  half_w; -len, -half_w,  half_w;
           0, -half_w, -half_w;    0,  half_w, -half_w;    0,  half_w,  half_w;    0, -half_w,  half_w
    ];

    vert_hom = [vertices_local, ones(8,1)]';
    vert_world = T * vert_hom;
    v = vert_world(1:3, :)';

    faces = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];

    patch('Vertices', v, 'Faces', faces, 'FaceColor', color, 'EdgeColor', 'k', 'FaceAlpha', 0.8);

end
