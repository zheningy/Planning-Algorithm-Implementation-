function[newtargetpos] = targetplanner(envmap, res, robotpos, targetpos, basepos, movetime);

dX = [-res  0 0 res];
dY = [ 0 -res res 0];


%failed to find an acceptable move
newtargetpos = targetpos;

for mind = 1:movetime
    %generate a move at random in 4 directions
    maxdist = 0;
    for iter = 1:2
        dir = ceil(4*random('unif', 0.00001, 1));    
        newx = targetpos(1) + dX(dir);
        newy = targetpos(2) + dY(dir);

        xdisc = fix(newx / res);
        ydisc = fix(newy / res);
        if (xdisc >= 1 & xdisc <= size(envmap, 1) & ydisc >= 1 & ydisc <= size(envmap, 2))
            dist = sqrt((newx-basepos(1))^2 + (newy-basepos(2))^2);
            if (envmap(xdisc, ydisc) == 0 & dist > maxdist)                
                newtargetpos(1) = newx;
                newtargetpos(2) = newy;
                maxdist = dist;
            end;
        end;
    end;
end;