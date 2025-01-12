function plotClickCallback(src, ~)
global hInputX hInputY
    pt = get(src, 'CurrentPoint');
    x = pt(1,1);
    y = pt(1,2);

    set(hInputX, 'String', num2str(x));
    set(hInputY, 'String', num2str(y));
end