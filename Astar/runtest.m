function[numofmoves, caught] = runtest(mapfile, robotstart, targetstart)

envmap = load(mapfile);

close all;

%draw the environment
image(envmap'*255);

%current positions of the target and robot
robotpos = robotstart;
targetpos = targetstart;

%load motion primitives
[mprim, res, num_angles] = loadmprim('unicycle_8angles.mprim');

%project start pos to the centre of the cell
robotpos = fix(robotpos/res) * res;
%now comes the main loop
hr = -1;
ht = -1;
numofmoves = 0;
caught = 0;
motion = zeros(size(mprim,3), size(mprim,4));
for i = 1:2000

    %draw the positions
    if (hr ~= -1)
        delete(hr);
        delete(ht);
    end;
    
    if i > 1
        for pt = 1 : size(motion, 1)
            hr = text(fix(motion(pt,1)/res), fix(motion(pt,2)/res), 'R');
            ht = text(targetpos(1)/res, targetpos(2)/res, 'T');
            pause(0.02);
            delete(hr);
            delete(ht);
        end
    end
    
    %call robot planner to find what they want to do
    t0 = clock;
    mprim_id = robotplanner(envmap, res, robotpos, targetpos, mprim);
    
    %get direction index for robotpos w.r.t the motion primitives
    normalized_angle = wrapTo2Pi(robotpos(3));
    dir = fix(normalized_angle / (2*pi / num_angles)) + 1;
    [ret, motion] = applyaction(envmap, res, robotpos, mprim, dir, mprim_id);
    newrobotpos = motion(end,:);
    if (ret ~= 1)
        fprintf(1, 'planned action leads to collision\n');
        return
    end
    
    %compute movetime for the target
    movetime = max(1, ceil(etime(clock,t0)));
    
    %call target planner to see how they move within the robot planning
    %time
    newtargetpos = targetplanner(envmap, res, robotpos, targetpos, targetstart, movetime);
       
    %make the moves
    robotpos = newrobotpos;
    targetpos = newtargetpos;
    numofmoves = numofmoves + 1;
    
    %check if target is caught
    thresh = 0.5;
    if (abs(robotpos(1)-targetpos(1)) <= thresh & abs(robotpos(2)-targetpos(2)) <= thresh)
        caught = 1;
        break;
    end;
    
end;

fprintf(1, 'target caught=%d number of moves made=%d\n', caught, numofmoves);