% Communicaiton between Arduino and MATLAB
%   @author         Alejandro Granados
%   @organisation   King's College London
%   @module         Medical Robotics Hardware Development
%   @year           2024

function T = forward_kinematics(r1, r2, t1, t2)
    t1 = deg2rad(t1);
    t2 = deg2rad(t2);
    
    A1 = [cos(t1) -sin(t1) 0 r1*cos(t1);
          sin(t1) cos(t1) 0 r1*sin(t1);
          0 0 1 0;
          0 0 0 1];

    A2 = [cos(t2) -sin(t2) 0 r2*cos(t2);
          sin(t2) cos(t2) 0 r2*sin(t2);
          0 0 1 0;
          0 0 0 1];

    T = A1*A2;
end
