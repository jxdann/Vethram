% Prompt user to select the CSV file
[filename, filepath] = uigetfile('*.csv', 'Select the MPU6050 data CSV file');
if isequal(filename, 0)
    disp('No file selected. Exiting...');
    return;
end

% Read the CSV file
fullPath = fullfile(filepath, filename);
data = readtable(fullPath);

% Extract data from the table
time = data.Time;
yaw = data.Yaw;
pitch = data.Pitch;
roll = data.Roll;

% Create a figure with subplots
figure('Name', 'MPU6050 Data Visualization', 'Position', [100, 100, 800, 600]);

% Plot Yaw
subplot(3, 1, 1);
plot(time, yaw, 'r-');
title('Yaw over Time');
xlabel('Time (s)');
ylabel('Yaw (degrees)');
grid on;

% Plot Pitch
subplot(3, 1, 2);
plot(time, pitch, 'g-');
title('Pitch over Time');
xlabel('Time (s)');
ylabel('Pitch (degrees)');
grid on;

% Plot Roll
subplot(3, 1, 3);
plot(time, roll, 'b-');
title('Roll over Time');
xlabel('Time (s)');
ylabel('Roll (degrees)');
grid on;

% Adjust the layout
sgtitle('MPU6050 Sensor Data');
set(gcf, 'Color', 'white');

% Add data statistics
%stats = sprintf('Data Statistics:\n\nYaw - Mean: %.2f°, Max: %.2f°, Min: %.2f°\nPitch - Mean: %.2f°, Max: %.2f°, Min: %.2f°\nRoll - Mean: %.2f°, Max: %.2f°, Min: %.2f°', ...
%    mean(yaw), max(yaw), min(yaw), ...
%    mean(pitch), max(pitch), min(pitch), ...
%    mean(roll), max(roll), min(roll));

%annotation('textbox', [0.02, 0.02, 0.3, 0.3], 'String', stats, 'EdgeColor', 'none', 'FitBoxToText', 'on');

% Display the plot
disp('Plotting complete. Close the figure window to exit.');