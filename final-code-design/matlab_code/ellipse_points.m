function [x,y] = ellipse_points(V,D, px, py)
    V1 = V(:,1);
    V2 = V(:,2);

    % Get eigenvalue 1
    d1 = sqrt(D(1,1));
    if d1 > 300
        d1 = 300;
    elseif d1 < -300
        d1 = -300;
    end

    % Get eigenvalue 2
    d2 = sqrt(D(2,2));
    if d2 > 300
        d2 = 300;
    elseif d1 < -300
        d2 = -300;
    end

    angle = atan2(V1(2), V1(1));

    % Generate ellipse points
    theta = linspace(0, 2*pi, 100);
    x_ellipse = d1 * cos(theta);
    y_ellipse = d2 * sin(theta);

    % Rotate the points
    x_rotated = x_ellipse * cos(angle) - y_ellipse * sin(angle);
    y_rotated = x_ellipse * sin(angle) + y_ellipse * cos(angle);

    % Translate the points
    x = x_rotated + px;
    y = y_rotated + py;
end