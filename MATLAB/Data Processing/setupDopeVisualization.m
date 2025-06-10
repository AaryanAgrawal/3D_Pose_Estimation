

function setupDopeVisualization(camera)
    % Setup 3D plot with camera coordinate conventions
    if nargin < 1, camera = false; end
    
    hold on; grid on; axis equal;
    xlabel('X (Right)'); ylabel('Y (Down)'); zlabel('Z (Forward)');
    title('Object Pose Visualization');
    view(45, 30);
    
    if camera
        % Camera coordinate frame origin
        plotTriadEuler([0, 0, 0], [0, 0, 0], 0.05, 'Camera');
    end
end