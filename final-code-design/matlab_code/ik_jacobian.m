% Compute Jacobian
%   @author         Mirroyal Ismayilov
%   @organisation   King's College London
%   @module         Applied Medical Robotics
%   @year           2024
function J = ik_jacobian(r1,r2,t1,t2)
N = 2;  % Number of joints
J = zeros(2, N);  % Initialize the Jacobian matrix

% t1 = deg2rad(t1);
% t2 = deg2rad(t2);

J = [ -r1*sin(t1) - r2*sin(t1 + t2), -r2*sin(t1 + t2);
      r1*cos(t1) + r2*cos(t1 + t2),  r2*cos(t1 + t2)];
end

