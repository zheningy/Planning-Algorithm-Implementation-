function [ ret, motion ] = applyaction(envmap, res, currentpos, mprim, dir, mprim_id)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

motion = zeros(size(mprim,3), size(mprim,4));
for i = 1 : size(mprim,3)
    newpos(1) = currentpos(1) + mprim(dir, mprim_id, i, 1);
    newpos(2) = currentpos(2) + mprim(dir, mprim_id, i, 2);
    newpos(3) = mprim(dir, mprim_id, i, 3);
    motion(i,:) = newpos;
    
    % check validity
    gridpos = newpos(1:2) / res;
    if (fix(gridpos(1)) < 1 | fix(gridpos(1)) > size(envmap, 1) | ...
            fix(gridpos(2)) < 1 | fix(gridpos(2)) > size(envmap, 2))
        ret = 0;
        return;
    end
    if ((envmap(fix(gridpos(1)), fix(gridpos(2))) ~= 0))
        ret = 0;
        return;
    end
        
end
ret = 1;

end




