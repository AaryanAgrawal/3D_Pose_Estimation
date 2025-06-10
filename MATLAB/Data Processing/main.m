clear all; close all; clc

% Parse detections (keep your existing functions)
[milk_pos, milk_ori, milk_size] = parse_dope_detections('Milk_detections.txt');
[oj_pos, oj_ori, oj_size] = parse_dope_detections('OrangeJuice_detections.txt');

% Reorder columns: [4, 1, 2, 3]
milk_ori = milk_ori(:, [4, 1, 2, 3]);
oj_ori = oj_ori(:, [4, 1, 2, 3]);

% Convert to Euler angles for averaging
milk_euler = quat2eul(milk_ori);
oj_euler = quat2eul(oj_ori);

% Calculate averages
milk_avg_pos = mean(milk_pos);
milk_avg_euler = mean(milk_euler);
oj_avg_pos = mean(oj_pos);
oj_avg_euler = mean(oj_euler);

milk_avg_euler_deg = rad2deg(milk_avg_euler)
oj_avg_euler_deg = rad2deg(oj_avg_euler)

% Relative position: milk relative to orange juice
rel_pos = milk_avg_pos - oj_avg_pos;

% Relative orientation: difference in Euler angles
rel_euler = milk_avg_euler - oj_avg_euler;

% Display results
% fprintf('Relative position (milk to OJ): [%.3f, %.3f, %.3f] m\n', rel_pos);
% fprintf('Relative orientation: [%.1f, %.1f, %.1f] degrees\n', rad2deg(rel_euler));

% Convert to OJ's local frame
oj_R = eul2rotm(oj_avg_euler, 'ZYX');
milk_R = eul2rotm(milk_avg_euler, 'ZYX');

% Relative position in OJ frame
rel_pos_local = (oj_R' * rel_pos')';

% Relative rotation in OJ frame  
rel_R_local = oj_R' * milk_R;
rel_euler_local = rotm2eul(rel_R_local, 'ZYX');

% Display local frame results
fprintf('Relative position in OJ frame: [%.3f, %.3f, %.3f] m\n', rel_pos_local);
fprintf('Relative orientation in OJ frame: [%.1f, %.1f, %.1f] degrees\n', rad2deg(rel_euler_local));

%% 3D visualization with averaged orientations
figure;
setupDopeVisualization();

% Plot average positions as scatter points
h1 = scatter3(milk_avg_pos(1), milk_avg_pos(2), milk_avg_pos(3), 200, 'r', 'filled');
h2 = scatter3(oj_avg_pos(1), oj_avg_pos(2), oj_avg_pos(3), 200, 'b', 'filled');

% Plot triads and bounding boxes using Euler angles
plotTriadEuler(milk_avg_pos, milk_avg_euler, 0.1, 'Milk');
plotTriadEuler(oj_avg_pos, oj_avg_euler, 0.1, 'OJ');
% 
% plotBoundingBoxEuler(milk_avg_pos, milk_avg_euler, milk_avg_size, 'r', 2);
% plotBoundingBoxEuler(oj_avg_pos, oj_avg_euler, oj_avg_size, 'b', 2);


%% RMS TIP = 1.457 mm


%% Read Polaris CSV files
milk_data = readtable('milk.csv');
oj_data = readtable('oj.csv');

% Extract tip positions in meters
milk_tips = [milk_data.tx_tip, milk_data.ty_tip, milk_data.tz_tip] / 1000;
oj_tips = [oj_data.tx_tip, oj_data.ty_tip, oj_data.tz_tip] / 1000;

% Calculate quadrilateral centers
milk_center = mean(milk_tips);
oj_center = mean(oj_tips);

% Calculate triads
[milk_R, milk_euler] = createTriad(milk_tips, milk_center);
[oj_R, oj_euler] = createTriad(oj_tips, oj_center);

% Relative transformation
rel_position = milk_center - oj_center;
rel_euler = milk_euler - oj_euler;
% Transform relative position to OJ's local frame
rel_position_local = oj_R' * rel_position';
rel_position_local = rel_position_local';
% Relative rotation in OJ's local frame
rel_R_local = oj_R' * milk_R;
rel_euler_local_polaris = rotm2eul(rel_R_local, 'ZYX');

% Display results
fprintf('Milk center: [%.4f, %.4f, %.4f] m\n', milk_center);
fprintf('OJ center: [%.4f, %.4f, %.4f] m\n', oj_center);
fprintf('Relative position: [%.4f, %.4f, %.4f] m\n', rel_position_local);
fprintf('Milk Euler: [%.2f, %.2f, %.2f] deg\n', rad2deg(milk_euler));
fprintf('OJ Euler: [%.2f, %.2f, %.2f] deg\n', rad2deg(oj_euler));
fprintf('Relative Euler: [%.2f, %.2f, %.2f] deg\n', rad2deg(rel_euler_local));

%% Plot both OJ frame triads for comparison
figure;
setupDopeVisualization();

% Plot OJ triad from DOPE data
% plotTriadEuler(oj_avg_pos, oj_avg_euler, 0.1, 'OJ-DOPE');
% scatter3(oj_avg_pos(1), oj_avg_pos(2), oj_avg_pos(3), 200, 'b', 'filled');

% Plot OJ triad from Polaris data  
plotTriadEuler(oj_center, oj_euler, 0.1, 'OJ-Polaris');
scatter3(oj_center(1), oj_center(2), oj_center(3), 200, 'c', 'filled');

% Plot OJ triad from Polaris data  
plotTriadEuler(milk_center, milk_euler, 0.1, 'Milk-Polaris');
scatter3(milk_center(1), milk_center(2), milk_center(3), 200, 'c', 'filled');

title('OJ Frame Comparison: DOPE vs Polaris');

%% %% Evaluation Metrics

% Calculate object diameters (hypotenuse of 2nd and 3rd dimensions)
milk_diameter = sqrt(milk_size(2)^2 + milk_size(3)^2);
oj_diameter = sqrt(oj_size(2)^2 + oj_size(3)^2);
avg_diameter = mean([milk_diameter, oj_diameter]);

% Translation error between DOPE and Polaris
trans_error = abs(rel_pos_local - rel_position_local);

% Rotation error between DOPE and Polaris  
rot_error = abs(rad2deg(rel_euler_local - rel_euler_local_polaris));

% Check criteria
diameter_10pct = 0.1 * avg_diameter;
trans_under_10pct = all(trans_error < diameter_10pct);
trans_under_5cm = all(trans_error < 0.05);
rot_under_5deg = all(rot_error < 5);

fprintf('\n=== EVALUATION METRICS ===\n');
fprintf('Average diameter: %.4f m\n', avg_diameter);
fprintf('Translation error: [%.4f, %.4f, %.4f] m\n', trans_error);
fprintf('Rotation error: [%.2f, %.2f, %.2f] deg\n', rot_error);