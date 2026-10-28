% Prompt the user to select the operating mode
modeChoice = questdlg('Which system do you want to run?', ...
    'System Launch', ...
    'Simulation Only', 'Hardware Only', 'Run Both', 'Simulation Only');

% Save this choice to the MATLAB workspace so the GUI can see it
assignin('base', 'ModeSelection', modeChoice);

% If the user closes the pop-up without clicking, stop the script
if isempty(modeChoice)
    error('Launch canceled by user.');
end



%Open App
myApp = DroneGCS;

% Get the Simulink model
modelName = 'Main_with_GuiG';
load_system(modelName);


targetBlock = [modelName, '/GUI_Telemetry'];

%Start the Simulation
if ~strcmp(modeChoice, 'Hardware Only')
set_param(modelName, 'SimulationCommand', 'start');
end 

%  Wait for the Simulink engine to finish compiling and begin executing
while ~strcmp(get_param(modelName, 'SimulationStatus'), 'running')
    pause(0.05); 
end

% 6. NOW attach the listener to the running block
hListener = add_exec_event_listener(targetBlock, 'PostOutputs', @(block, event) updateGUI(block, event, myApp));

% --- The Callback Function ---
function updateGUI(block, event, app)
    % Get the current simulation time
    currentTime = block.CurrentTime;
    
    % Read the live values off the input ports of the GUI_Telemetry block
    currentU      = block.InputPort(1).Data;
    currentStates = block.InputPort(2).Data;
    currentGPSPos = block.InputPort(3).Data;
    currentGPSVel = block.InputPort(4).Data;
    currentIMU    = block.InputPort(5).Data;
    currentMagYaw = block.InputPort(6).Data;
    
    % Increment the fast counter (runs every 0.005s solver step)
    app.FastCounter = app.FastCounter + 1;
    
    % Down-sample: Only update the UI every 10th step (20 Hz refresh rate)
    if mod(app.FastCounter, 10) == 0
        
        % ------ Append data to buffers ---
        app.TimeBuffer = [app.TimeBuffer, currentTime];
        
        % Transpose data to rows if necessary to match plot formatting
        app.UBuffer = [app.UBuffer; currentU'];
        
        % Extract X,Y,Z (Index 1,2,3) and Vx,Vy,Vz (Index 7,8,9) from the 12-state vector
        app.StatePosBuffer = [app.StatePosBuffer; currentStates(1:3)'];
        app.StateVelBuffer = [app.StateVelBuffer; currentStates(7:9)'];
        app.StateYawBuffer = [app.StateYawBuffer; currentStates(6)'];
        
        app.GPSPosBuffer   = [app.GPSPosBuffer; currentGPSPos'];
        app.GPSVelBuffer   = [app.GPSVelBuffer; currentGPSVel'];
        app.IMUAccelBuffer = [app.IMUAccelBuffer; currentIMU'];
        app.MagYawBuffer   = [app.MagYawBuffer; currentMagYaw];

% --- Controller Tab Plots ---
        
        % Control Effort
        plot(app.UIAxes, app.TimeBuffer, app.UBuffer);
        legend(app.UIAxes, {'Thrust', 'Roll', 'Pitch', 'Yaw'}, 'Location', 'best');
        
        % State Position
        plot(app.UIAxes2, app.TimeBuffer, app.StatePosBuffer);
        legend(app.UIAxes2, {'x', 'y', 'z'}, 'Location', 'best');
        
        % State Velocity
        plot(app.UIAxes3, app.TimeBuffer, app.StateVelBuffer);
        legend(app.UIAxes3, {'vx', 'vy', 'vz'}, 'Location', 'best');
        
        % State Attitude (Yaw)
        plot(app.UIAxes4, app.TimeBuffer, app.StateYawBuffer);
        legend(app.UIAxes4, {'Yaw (\psi)'}, 'Location', 'best');
        
        
        % --- Sensor Tab Plots ---
        
        % Sensor GPS Position
        plot(app.UIAxes_2, app.TimeBuffer, app.GPSPosBuffer);
        legend(app.UIAxes_2, {'x_{gps}', 'y_{gps}', 'z_{baro}'}, 'Location', 'best');
        
        % Sensor GPS Velocity
        plot(app.UIAxes2_2, app.TimeBuffer, app.GPSVelBuffer);
        legend(app.UIAxes2_2, {'vx_{gps}', 'vy_{gps}', 'vz_{gps}'}, 'Location', 'best');
        
        % Sensor IMU Acceleration
        plot(app.UIAxes3_2, app.TimeBuffer, app.IMUAccelBuffer);
        legend(app.UIAxes3_2, {'ax', 'ay', 'az'}, 'Location', 'best');
        
        % Sensor Magnetometer Yaw
        plot(app.UIAxes4_2, app.TimeBuffer, app.MagYawBuffer);
        legend(app.UIAxes4_2, {'Yaw_{mag}'}, 'Location', 'best');
        
        % Force the UI to refresh visually without freezing Simulink
        drawnow limitrate;
    end
end
