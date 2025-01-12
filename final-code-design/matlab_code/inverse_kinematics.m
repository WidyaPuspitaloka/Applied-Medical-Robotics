function [theta1, theta2] = inverse_kinematics(r1, r2, px, py)
    theta2 = acos(((px^2) + (py^2)-(r1^2)-(r2^2))/(2*r1*r2));
    theta1 = atan2(py,px) - atan2((r2*sin(theta2)),(r1+(r2*cos(theta2))));

    theta2 = rad2deg(theta2);
    theta1 = rad2deg(theta1);
end