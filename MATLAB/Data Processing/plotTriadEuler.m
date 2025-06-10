%% Euler-based Visualization Functions
function plotTriadEuler(pos, euler, scale, label)
    % Plot coordinate triad using Euler angles [roll, pitch, yaw]
    R = eul2rotm(euler, 'ZYX');
    
    % Object frame axes in camera coordinates
    x_axis = R(:,1) * scale;
    y_axis = R(:,2) * scale; 
    z_axis = R(:,3) * scale;
    
    % Plot RGB triad (X=red, Y=green, Z=blue)
    quiver3(pos(1), pos(2), pos(3), x_axis(1), x_axis(2), x_axis(3), ...
            'r', 'LineWidth', 3, 'MaxHeadSize', 0.3);
    quiver3(pos(1), pos(2), pos(3), y_axis(1), y_axis(2), y_axis(3), ...
            'g', 'LineWidth', 3, 'MaxHeadSize', 0.3);
    quiver3(pos(1), pos(2), pos(3), z_axis(1), z_axis(2), z_axis(3), ...
            'b', 'LineWidth', 3, 'MaxHeadSize', 0.3);
    
    % Add text label
    text(pos(1), pos(2), pos(3)+0.05, label, 'FontSize', 10, 'FontWeight', 'bold');
end