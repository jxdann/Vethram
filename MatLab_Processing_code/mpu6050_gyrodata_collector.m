% Define the serial port (adjust based on your COM port)
serialObj = serialport('COM3', 115200); % Replace 'COM3' with your actual serial port
configureTerminator(serialObj, "LF"); % Line terminator
serialObj.Timeout = 0.1; % Set a timeout period of 0.1 seconds

% Initialize arrays to store data
timeData = [];
yawData = [];
pitchData = [];
rollData = [];

% Set the duration for data collection (30 seconds)
duration = 30;
startTime = datetime('now');

disp('Starting data collection for 30 seconds...');

% Main loop for data collection
while seconds(datetime('now') - startTime) < duration
    try
        data = readline(serialObj); % Read the incoming data
        
        % Remove the 'ypr' prefix if it exists
        if contains(data, 'ypr')
            data = erase(data, 'ypr');
        end
        
        % Replace the special characters with spaces for easier splitting
        data = strrep(data, '→', ' ');
        data = strrep(data, '←', ' ');
        
        % Check if data is valid and split it
        if ~isempty(data)
            values = str2double(split(strtrim(data))); % Split and convert to numbers
            
            % Check if exactly 3 values (y, p, r) are received
            if numel(values) == 3
                timeData(end+1) = seconds(datetime('now') - startTime);
                yawData(end+1) = values(1);
                pitchData(end+1) = values(2);
                rollData(end+1) = values(3);
                
                % Display progress
                if mod(length(timeData), 10) == 0
                    disp(['Collected ' num2str(length(timeData)) ' data points...']);
                end
            else
                disp('Invalid data format received.');
            end
        else
            disp('Invalid or empty data received');
        end
    catch ME
        disp(['Error: ', ME.message]); % Display any error that occurs
    end
    
    % Short pause to avoid overloading the CPU
    pause(0.01);
end

% Close the serial port
clear serialObj;

% Create a table with the collected data
dataTable = table(timeData', yawData', pitchData', rollData', ...
                  'VariableNames', {'Time', 'Yaw', 'Pitch', 'Roll'});

% Generate a filename with the current date and time
filename = ['MPU6050_Data_' datestr(now, 'yyyymmdd_HHMMSS') '.csv'];

% Write the data to a CSV file
writetable(dataTable, filename);

disp(['Data collection complete. Data saved to ' filename]);