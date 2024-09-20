% MATLAB Code to replicate the Processing sketch

clear;
clc;

% Setup serial communication
port = serialport("COM3", 115200);  % Modify COM port accordingly
configureTerminator(port, "LF");

% Initialize variables
synced = 0;
serialCount = 0;
sensorPacket = zeros(1, 41, 'uint8');
interval = tic;

% Quaternions
q1 = [1, 0, 0, 0];
q2 = [1, 0, 0, 0];
q3 = [1, 0, 0, 0];
q4 = [1, 0, 0, 0];

% Setup figure for 3D rendering
figure;
hold on;
axis equal;
grid on;
view(3);

% Main loop for drawing and serial communication
while true
    % Check if serial data is available
    if port.NumBytesAvailable > 0
        % Read incoming data
        data = read(port, port.NumBytesAvailable, 'uint8');
        
        % Process incoming data
        for i = 1:length(data)
            ch = data(i);
            if synced == 0 && ch ~= uint8('$')
                continue;  % Wait for sync character
            end
            synced = 1;
            
            % Check for packet alignment issues
            if (serialCount == 1 && ch ~= uint8(1)) || ...
               (serialCount == 39 && ch ~= uint8(13)) || ...
               (serialCount == 40 && ch ~= uint8(10))
                serialCount = 0;
                synced = 0;
                continue;
            end
            
            % Store the data
            sensorPacket(serialCount + 1) = ch;
            serialCount = serialCount + 1;
            
            % If full packet is received, process it
            if serialCount == 41
                serialCount = 0;
                
                % Extract quaternion values
                q1 = extractQuaternion(sensorPacket, 2);
                q2 = extractQuaternion(sensorPacket, 11);
                q3 = extractQuaternion(sensorPacket, 20);
                q4 = extractQuaternion(sensorPacket, 29);
                
                % Print quaternion data
                fprintf('Sensor 1 Quaternion: %.4f, %.4f, %.4f, %.4f\n', q1);
                fprintf('Sensor 2 Quaternion: %.4f, %.4f, %.4f, %.4f\n', q2);
                fprintf('Sensor 3 Quaternion: %.4f, %.4f, %.4f, %.4f\n', q3);
                fprintf('Sensor 4 Quaternion: %.4f, %.4f, %.4f, %.4f\n', q4);
                
                % Plot the boxes with updated orientations
                cla;
                plotBox(q1, [-1, 0, 0]);
                plotBox(q2, [1, 0, 0]);
                plotBox(q3, [0, -1, 0]);
                plotBox(q4, [0, 1, 0]);
                drawnow;
            end
        end
    end
    
    % Periodically send trigger to the MPU
    if toc(interval) > 1
        write(port, 'r', 'char');
        interval = tic;
    end
end

%% Helper function to extract quaternion from sensor packet
function q = extractQuaternion(packet, startIdx)
    q = zeros(1, 4);
    for i = 1:4
        q(i) = double(typecast(uint16(bitshift(packet(startIdx + (i-1)*2), 8) + packet(startIdx + (i-1)*2 + 1)), 'int16')) / 16384.0;
        if q(i) >= 2
            q(i) = q(i) - 4;
        end
    end
end

%% Helper function to plot a 3D box with a given quaternion
function plotBox(q, pos)
    % Convert quaternion to rotation matrix
    R = quat2rotm(q);
    
    % Define a cube
    [X, Y, Z] = ndgrid([-1, 1], [-1, 1], [-1, 1]);
    
    % Rotate and translate the cube
    verts = [X(:), Y(:), Z(:)] * 50 * R' + pos * 100;
    
    % Plot the cube as patches
    patch('Vertices', verts, 'Faces', convhull(verts), ...
        'FaceColor', 'red', 'FaceAlpha', 0.6, 'EdgeColor', 'none');
end
