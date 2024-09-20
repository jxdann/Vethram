% Define joint positions in 3D space (x, y, z coordinates)
% The body is now standing upright on the XY plane (z = 0 is the ground level)
joints = [
    0, 0, 0;       % Hip (at the origin)
    0, 0, 0.5;     % Spine (above the hip, along the Z-axis)
    0, 0, 1;       % Neck (higher up along Z-axis)
    0.3, 0, 1.2;   % Right Shoulder (to the right, slightly up)
    -0.3, 0, 1.2;  % Left Shoulder (to the left, slightly up)
    0.4, 0, 0.9;   % Right Elbow (slightly down from the shoulder)
    -0.4, 0, 0.9;  % Left Elbow (slightly down from the shoulder)
    0.5, 0, 0.6;   % Right Hand (lower)
    -0.5, 0, 0.6;  % Left Hand (lower)
    0.2, 0, -0.5;  % Right Knee (below the hip, along the Z-axis)
    -0.2, 0, -0.5; % Left Knee (below the hip, along the Z-axis)
    0.2, 0, -1;    % Right Foot (ground level)
    -0.2, 0, -1;   % Left Foot (ground level)
];

% Connect joints with lines (segments of the body)
connections = [
    1, 2;  % Hip to Spine
    2, 3;  % Spine to Neck
    3, 4;  % Neck to Right Shoulder
    3, 5;  % Neck to Left Shoulder
    4, 6;  % Right Shoulder to Right Elbow
    5, 7;  % Left Shoulder to Left Elbow
    6, 8;  % Right Elbow to Right Hand
    7, 9;  % Left Elbow to Left Hand
    1, 10; % Hip to Right Knee
    1, 11; % Hip to Left Knee
    10, 12; % Right Knee to Right Foot
    11, 13; % Left Knee to Left Foot
];

% Create 3D plot
figure;
hold on;
axis equal;
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Skeleton Standing on XY Plane');

% Plot joints as red circles
for i = 1:size(joints, 1)
    plot3(joints(i,1), joints(i,2), joints(i,3), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
end

% Plot connections as lines between joints
for i = 1:size(connections, 1)
    joint1 = joints(connections(i,1), :);
    joint2 = joints(connections(i,2), :);
    plot3([joint1(1), joint2(1)], [joint1(2), joint2(2)], [joint1(3), joint2(3)], 'b-', 'LineWidth', 2);
end

view(3);  % Set view to 3D
hold off;
