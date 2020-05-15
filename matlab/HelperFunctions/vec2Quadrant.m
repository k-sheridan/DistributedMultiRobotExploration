function [quadrant] = vec2Quadrant(vec)
% converts 2d vec to a quadrant number

if vec(1)>=0
    if vec(2)>=0
        % +, + => 1
        quadrant = 1;
    else
        % +, - => 4
        quadrant = 4;
    end
else
    if vec(2)>=0
        % -, + => 2
        quadrant = 2;
    else
        % -, - => 3
        quadrant = 3;
    end
end

end

