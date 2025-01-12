% Communicaiton between Arduino and MATLAB
%   @author         Alejandro Granados
%   @organisation   King's College London
%   @module         Medical Robotics Hardware Development
%   @year           2023

close all
clear all

% declare global variables
%   s           serial port communication
%   hInput1     input widget 1
%   hInput1     input widget 2
%   hPlot       plot widget
%   hFig        figure widget
%   hTimer      continuous timer
%   c           command type
%   y1          data stream series 1
%   y2          data stream series 2
global s hInputX hInputY hPlot hFig hTimer c y1 y2 r1 r2 demandedDegrees currentDegrees counter arm1 arm2 ellipse endEffectorPosition

%% Set up
% Create serial port object
s = serialport('/dev/cu.usbmodem34B7DA6256202', 9600);
counter = 1;

configureTerminator(s,"CR/LF");
s.UserData = struct("Data",[],"Count",1);

% Create GUI
hFig = figure;

% Define fixed values for r1 and r2
r1 = 78;
r2 = 78;

% % Create input field for sending commands to microcontroller
% hInput1 = uicontrol('Style', 'edit', 'Position', [20, 20, 100, 25]);
% hInput2 = uicontrol('Style', 'edit', 'Position', [120, 20, 100, 25]);
% 
% % Create button for sending commands
% hSend = uicontrol('Style', 'pushbutton', 'String', 'Send', 'Position', [20, 50, 100, 25], 'Callback', @sendCommand);
% 
% % Create plot area
% hPlot = axes('Position', [0.2, 0.6, 0.6, 0.3]);

% Create label and input field for X coordinate
hLabelX = uicontrol('Style', 'text', 'String', 'X:', 'Position', [20, 20, 20, 25]);
hInputX = uicontrol('Style', 'edit', 'Position', [40, 20, 100, 25]);

% Create label and input field for Y coordinate
hLabelY = uicontrol('Style', 'text', 'String', 'Y:', 'Position', [150, 20, 20, 25]);
hInputY = uicontrol('Style', 'edit', 'Position', [170, 20, 100, 25]);

% Create button for sending coordinates
hSend = uicontrol('Style', 'pushbutton', 'String', 'Send', 'Position', [280, 20, 100, 25], 'Callback', @sendCommand);

% Create plot area
hPlot = axes('Position', [0.2, 0.3, 0.6, 0.6], 'ButtonDownFcn', @plotClickCallback);
hold on
plot(hPlot, 0, 0, 'ro', 'LineWidth', 3);
rectangle('Position', [-78, 0, 156, 156], 'EdgeColor', 'b', 'Parent', hPlot)

% Draw arm
arm1 = line([0; r1], [0; 0], 'Parent', hPlot, 'Color', 'black', 'LineWidth', 3);
arm2 = line([r1; r1+r2], [0; 0], 'Parent', hPlot, 'Color', 'black', 'LineWidth', 3);

% Draw manipulation ellipse
[V,D] = manipulation_eig(r1, r2, 0, 0);
[ellipse_x, ellipse_y] = ellipse_points(V, D, r1 + r2, 0);
ellipse = plot(ellipse_x, ellipse_y, 'g');


% Set up variables for real-time plotting
c = [];
y1 = [];
y2 = [];
t0 = datetime("now");

demandedDegrees = [];  % To store demanded joint angles
currentDegrees = [];   % To store current joint angles
endEffectorPosition = [];

% Set up timer for continuously receiving data from microcontroller
hTimer = timer('ExecutionMode', 'fixedRate', 'Period', 0.001, 'TimerFcn', @readDataTimer);
start(hTimer);
hFig.CloseRequestFcn = @closeGUI;


%% Callback function for sending commands
function sendCommand(~, ~)
global s hInputX hInputY r1 r2 demandedDegrees hPlot ellipse inputX inputY

    % Get values from input fields as strings but convert to numbers
    % input1 = str2num(get(hInput1, 'String'));
    % input2 = str2num(get(hInput2, 'String'));

    inputX = str2double(string(get(hInputX, 'String')));
    inputY = str2double(string(get(hInputY, 'String')));
    
    % validate input
    if numel(inputX) ~= 1
        return; % input field must contain one value
    end
    if numel(inputY) ~= 1
        return; % input field must contain one value
    end

    hold on
    plot(hPlot, inputX, inputY, 'r*');

    % Calculate joint angles using inverse kinematics
    [joint1, joint2] = inverse_kinematics(r1, r2, inputX, inputY);
    disp([joint1, joint2]);
    
    % Save demanded joint angles
    % demandedDegrees = [demandedDegrees; joint1, joint2];

    % Draw manipulation ellipse
    [V,D] = manipulation_eig(r1, r2, joint1, joint2);
    [ellipse_x, ellipse_y] = ellipse_points(V, D, inputX, inputY);
    set(ellipse, 'XData', ellipse_x, 'YData', ellipse_y);

    % format command values as a string, e.2fg. C40.0,3.5;
    cmdStr = sprintf("C%.2f,%.2f;", joint1, joint2);
    
    % Send command string to microcontroller
    write(s, cmdStr, "string");
end


%% Callback function fo reading time series values from microcontroller
function readDataTimer(~, ~)
global s hPlot c y1 y2 r1 r2 currentDegrees data_display counter arm1 arm2 endEffectorPosition inputX inputY

    % Read the ASCII data from the serialport object.
    dataStr = readline(s);
    if isempty(dataStr)
        return;
    end
    if dataStr == ""
        return;
    end

    % Parse data values from string and add accumulate into arrays
    % e.g. Arduino sending 2 series: c1,100
    data = sscanf(dataStr, "%c%f,%f,%f");
    %disp([counter; data]);
    

    % Calculate forward kinematics
    th1 = data(2);
    th2 = data(3);

    T = forward_kinematics(r1, r2, th1, th2);

    % Extract the end effector position from T (if T is a transformation matrix)
    endEffectorPositionX = T(1, 4); % X position
    endEffectorPositionY = T(2, 4); % Y position
    %disp([counter, endEffectorPositionX, endEffectorPositionY, data(4)]);
    counter = counter + 1;
    %endEffectorPosition = [endEffectorPosition; T(1, 4),  T(2, 4), inputX, inputY];
    %save('endEffectorPosition.mat', 'endEffectorPosition');

    % Real-time plotting
    % y1 = endEffectorPositionX; % the X position from forward kinematics
    % y2 = endEffectorPositionY; % the Y position from forward kinematics

    % currentDegrees = [currentDegrees; data(2), data(3)]; % Store current angles
    % save('currentDegrees.mat', 'currentDegrees');


    % Clear the plot and redraw for updated values
    % cla(hPlot); % Clear axes
    % plot(hPlot, y1, y1, 'r-'); % Plotting y1 vs y1
    % hold(hPlot, 'on');
    % plot(hPlot, y1, y2, 'b-'); % Plotting y1 vs y2
    % hold(hPlot, 'off');

    %plot(hPlot, y1, y1, 'ro', 'MarkerFaceColor', 'r'); % Plotting y1 vs y1 as red points
    % hold(hPlot, 'on');

    % plot(hPlot, y1, y2, 'bo'); % Plotting y1 vs y2 as blue points
    % hold(hPlot, 'on');

    % c = [c, data(1)];
    % y1 = [y1, data(2)]; 
    % y2 = [y2, data(3)];
    % currentDegrees = [currentDegrees; data(2), data(3)]; % Store current angles
    % 
    % configure callback 
    configureCallback(s, "off");
     
    % real-time plotting, e.g. 2 series (y1, y2)
    % plot(hPlot, y1, y2, 'bo'); % Plotting y1 vs y2 as blue points
    hold on
    % plot(hPlot, y1, y1, 'r-');
    % hold on
    % plot(hPlot, y1, y2, 'b-');

    % plot robotic arm
    r1x = r1 *cosd(th1);
    r1y = r1 *sind(th1);
    set(arm1, 'XData', [0;r1x], 'YData', [0;r1y]);
    set(arm2, 'XData', [r1x;endEffectorPositionX], 'YData', [r1y;endEffectorPositionY]);
    
end

%% Callback function for closing the GUI
function closeGUI(~, ~)
global s hFig hTimer

    % Stop timer
    stop(hTimer);
    delete(hTimer);
    
    % Close serial port
    %delete(s);
    
    % Close GUI
    delete(hFig);
end

   