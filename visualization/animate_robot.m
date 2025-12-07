function animate_robot(t_anim, x_anim, end_effector_path, target_pos, params, t_end, fps)
% ANIMATE_ROBOT Animates the 3-link robotic arm motion in 3D.
%
%   Inputs:
%       t_anim            : Animation time vector
%       x_anim            : Interpolated state vector at animation times
%       end_effector_path : Pre-calculated path of the end effector
%       target_pos        : Cell array of target positions
%       params            : Struct with robot parameters (a2, a3)
%       t_end             : Total simulation time
%       fps               : Frames per second for animation playback

%% 6. 3D Animation with Real-Time Sync
h_fig = figure('Name', '3D Animation');
axis_limit = params.a2 + params.a3 + 0.1;
axis([-axis_limit axis_limit -axis_limit axis_limit -axis_limit axis_limit]);
grid on; hold on;
xlabel('X'); ylabel('Y'); zlabel('Z');
view(135, 30); 
axis equal;

% Plot target locations as a green circle
for i = 1:length(target_pos)
    plot3(target_pos{i}(1), target_pos{i}(2), target_pos{i}(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', '#00CC00');
end

% Visual settings
link_width = 0.05; 
color_base = [0.2 0.2 0.2]; 
color_arm2 = [0   0   1];   
color_arm3 = [1   0   0];   

disp('Starting Animation...');
tic; % Start Real-World Timer

while ishandle(h_fig) % Loop until figure is closed or time runs out
    
    % 1. Get current real-world time
    t_current_real = toc;
    
    % 2. Stop if we exceed simulation duration
    if t_current_real > t_end
        break;
    end
    
    % 3. Find the closest index in our INTERPOLATED data
    % Formula: index = (time * fps) + 1
    idx = round(t_current_real * fps) + 1;
    
    % Safety check for index bounds
    if idx > length(t_anim)
        idx = length(t_anim);
    end
    
    % Get angles from interpolated data
    q1_val = x_anim(idx, 1);
    q2_val = x_anim(idx, 2);
    q3_val = x_anim(idx, 3);
    
    % --- KINEMATICS ---
    % A1 (Base to Shoulder)
    A1 = [cos(q1_val) 0 sin(q1_val) 0;
          sin(q1_val) 0 -cos(q1_val) 0;
          0 1 0 0;
          0 0 0 1];
    % A2 (Shoulder to Elbow)
    A2 = [cos(q2_val) -sin(q2_val) 0 params.a2*cos(q2_val);
          sin(q2_val)  cos(q2_val) 0 params.a2*sin(q2_val);
          0 0 1 0;
          0 0 0 1];
    % A3 (Elbow to Wrist)
    A3 = [cos(q3_val) -sin(q3_val) 0 params.a3*cos(q3_val);
          sin(q3_val)  cos(q3_val) 0 params.a3*sin(q3_val);
          0 0 1 0;
          0 0 0 1];
          
    % Global Transforms
    T1 = A1;         
    T2 = T1 * A2;    
    T3 = T2 * A3;    
    
    % --- DRAWING ---
    cla; % Clear previous frame
    
    % Draw the end effector trace up to the current point
    if idx > 1
        plot3(end_effector_path(1:idx, 1), end_effector_path(1:idx, 2), end_effector_path(1:idx, 3), 'm-', 'LineWidth', 1.5);
    end
    
    draw_link_3d(T1, 0.1, link_width*1.5, color_base);
    draw_link_3d(T2, params.a2, link_width, color_arm2);
    draw_link_3d(T3, params.a3, link_width, color_arm3);
    
    title(['Time: ' num2str(t_anim(idx), '%.2f') ' s / ' num2str(t_end) ' s']);
    axis([-axis_limit axis_limit -axis_limit axis_limit -axis_limit axis_limit]);

    
    drawnow;

end
end

function draw_link_3d(T, len, width, color)
% DRAW_LINK_3D Draws a 3D rectangular link attached to a frame
%   T:     4x4 Homogeneous Transformation Matrix (Where the link ends)
%   len:   Length of the link (a_i parameter)
%   width: Thickness of the link
%   color: RGB vector like [0 0 1] for blue

    % 1. Define the Box Geometry in Local Coordinates
    % In DH, the frame is at the END of the link.
    % So the link extends from x = -len to x = 0.
    
    half_w = width / 2;
    
    % The 8 Vertices of a box
    % Format: [x, y, z]
    vertices_local = [
        -len, -half_w, -half_w;  % 1. Back-Left-Bottom
        -len,  half_w, -half_w;  % 2. Back-Right-Bottom
        -len,  half_w,  half_w;  % 3. Back-Right-Top
        -len, -half_w,  half_w;  % 4. Back-Left-Top
           0, -half_w, -half_w;  % 5. Front-Left-Bottom
           0,  half_w, -half_w;  % 6. Front-Right-Bottom
           0,  half_w,  half_w;  % 7. Front-Right-Top
           0, -half_w,  half_w   % 8. Front-Left-Top
    ];

    % 2. Transform Vertices to World Coordinates
    % Multiply by T to move them into place
    
    % Convert to Homogeneous [x; y; z; 1]
    vert_hom = [vertices_local, ones(8,1)]'; 
    
    % Apply Transform
    vert_world = T * vert_hom;
    
    % Extract back to [x, y, z]
    v = vert_world(1:3, :)';

    % 3. Define the Faces (Connect the dots)
    faces = [
        1 2 6 5; % Bottom
        2 3 7 6; % Right
        3 4 8 7; % Top
        4 1 5 8; % Left
        1 2 3 4; % Back
        5 6 7 8  % Front
    ];

    % 4. Draw the Patch
    patch('Vertices', v, 'Faces', faces, ...
          'FaceColor', color, ...
          'EdgeColor', 'k', ... % Black edges
          'FaceAlpha', 0.8);    % Slight transparency
end