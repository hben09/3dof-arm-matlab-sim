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