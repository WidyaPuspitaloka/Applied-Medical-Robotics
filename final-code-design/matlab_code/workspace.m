% % Communicaiton between Arduino and MATLAB
% %   @author         Alejandro Granados
% %   @organisation   King's College London
% %   @module         Medical Robotics Hardware Development
% %   @year           2024

close all
clear all
global hPlot hFig c x y 

hFig = figure;
hPlot = axes('Position', [0.2, 0.35, 0.6, 0.6]);

c = 0;
x = [];
y = [];

r1 = 94; % length of robotic arm
r2 = 94;



resolution = 70;   
angle1_range = linspace(0, 180, resolution);  % Angles in degrees
angle2_range = linspace(0, 360, resolution);  % Angles in degrees

for t1 = 1:length(angle1_range)
    for t2 = 1:length(angle2_range)
        c = c+1;
        T = forward_kinematics(r1, r2, (angle1_range(t1)), (angle2_range(t2)));  
        x(c) = T(1,4);
        y(c) = T(2,4);
    end
end

colour = linspace(1,10,length(x));
scatter(hPlot, x, y, 20, colour, 'filled');
xlim([-(r1+r2),r1+r2]);
ylim([-(r1+r2),r1+r2]);
hold on


% Add a rectangle representing the fitted 156x156 space
rectangle('Position', [-78, 0, 156, 156], 'EdgeColor', 'black', 'LineWidth', 2);
% text(-75, -80, '156x156 Space', 'Color', 'r', 'FontSize', 10);

% Additional plot formatting
xlabel('X Coordinate');
ylabel('Y Coordinate');
% title('Workspace of Robotic Arm with Fitted 156x156 Space');
grid on;
axis equal;
hold off;