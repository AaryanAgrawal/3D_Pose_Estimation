% Create triads for each dataset
function [R, euler] = createTriad(tips, center)
    % Y axis: center to midpoint of vertices 1&4
    mid_14 = (tips(1,:) + tips(4,:)) / 2;
    y_axis = (mid_14 - center);
    y_axis = y_axis / norm(y_axis);
    
    % X axis: center to midpoint of vertices 1&2
    mid_12 = (tips(1,:) + tips(2,:)) / 2;
    x_axis = (mid_12 - center);
    x_axis = x_axis / norm(x_axis);
    
    % Z axis: perpendicular to X and Y
    z_axis = cross(x_axis, y_axis);
    z_axis = z_axis / norm(z_axis);
    
    % Orthogonalize (Gram-Schmidt)
    y_axis = cross(z_axis, x_axis);
    y_axis = y_axis / norm(y_axis);
    
    % Rotation matrix
    R = [z_axis; y_axis; x_axis]';
    
    % Convert to Euler angles
    euler = rotm2eul(R, 'ZYX');
end