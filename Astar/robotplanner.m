function[mprim_id] = robotplanner(envmap, res, robotpos, targetpos, mprim);

MEX = 0;

%failed to find an acceptable move
mprim_id = 1;   %arbitrary move

if (MEX == 1)
	%if using MEX, here you would call the planner
    robotpos = robotpos
	mprim_id = planner(envmap, robotpos, targetpos);
    mprim_id = mprim_id + 1;

else
	%otherwise do planning right here 

	%for now greedily move towards the target, 
	%but this is where you can put your planner 
% 	mindisttotarget = 1000000;
%     
%     %get direction index for robotpos w.r.t the motion primitives    
%     normalized_angle = wrapTo2Pi(robotpos(3));
%     dir = fix(normalized_angle / (2*pi / size(mprim, 1)) + 0.5) + 1;
% 	for idx = 1:size(mprim, 2)
%            [ret, motion] = applyaction(envmap, res, robotpos, mprim, dir, idx);
%            new_pos = motion(end,:);
%            if (ret == 1)    
%                if (new_pos(1) >= 1 & new_pos(1) <= size(envmap, 1) & new_pos(2) >= 1 & new_pos(2) <= size(envmap, 2))         
%                     disttotarget = sqrt((new_pos(1)-targetpos(1))^2 + (new_pos(2)-targetpos(2))^2);
%                     if(disttotarget < mindisttotarget)
%                       mindisttotarget = disttotarget;
%                       mprim_id = idx;
%                     end;
%                end;
%            end
% 	end;
    [w, h] = size(envmap);
    t = size(mprim, 1);

    robotpos(1:2) = [fix(robotpos(1)/res+0.5), fix(robotpos(2)/res+0.5)];
    normalized_angle = wrapTo2Pi(robotpos(3));
    robotpos(3) = fix(normalized_angle / (2*pi / size(mprim, 1)) + 0.5) + 1;

    targetpos(1:2) = [fix(targetpos(1)/res+0.5), fix(targetpos(2)/res+0.5)];

    closedSet = [];
    openSet = [robotpos];

    eps = 1;
    g = Inf(w, h, t);
    g(robotpos(1),robotpos(2),robotpos(3)) = 0;

    f = Inf(w, h, t);
    f(robotpos(1),robotpos(2),robotpos(3)) = eps * heuristic(robotpos, targetpos);
    cameFrom = cell(w, h, t);

    while ~isempty(openSet)
        linear_idx = sub2ind([w, h, t], openSet(:,1), openSet(:,2), openSet(:,3));
        [~, current] = min(f(linear_idx));
        [x, y, d] = ind2sub([w, h, t], linear_idx(current));
        if find(ismembertol([x,y], targetpos(1:2), 'ByRows', true), 1)
%             path = [];
            iter = cameFrom{x, y, d};
            while(1)
                next = cameFrom{iter(1), iter(2), iter(3)};
                if isempty(next)
                    mprim_id = iter(4);
                    break
                end
                if (all(ismembertol(next(1:2), robotpos(1:2), 'ByRows', true)))
                    mprim_id = next(4);
                    break
                end
                iter = next;
            end
            break
        end
        closedSet = [closedSet; [x y d]];
        
%         assert(sum(ismember(openSet, current, 'rows')) == 1);
        openSet(find(ismembertol(openSet, [x y d], 'ByRows', true), 1),:) = [];
    %     rand_idx = randperm(size(dir,1));
        for idx = 1:size(mprim, 2)
    %         i = rand_idx(j);
            [ret, motion] = applyaction(envmap, res, [x*res, y*res], mprim, d, idx);
            if ret == 1
                neighbour = motion(end,:);
                neighbour(1:2) = [fix(neighbour(1)/res+0.5), fix(neighbour(2)/res+0.5)];
                normalized_angle = wrapTo2Pi(neighbour(3));
                neighbour(3) = fix(normalized_angle / (2*pi / size(mprim, 1)) + 0.5) + 1;
                
                if sum(ismembertol(closedSet, neighbour, 'ByRows', true)) > 0
                    continue
                end
                if sum(ismembertol(openSet, neighbour, 'ByRows', true)) == 0
                    openSet = [openSet; neighbour];
                end
                
                score = g(x, y, d) + pdist([x,y;neighbour(1:2)], 'euclidean');
                if score > g(neighbour(1), neighbour(2), neighbour(3))
                    continue
                end
%                 if find(ismembertol(current(1:2), robotpos(1:2), 'ByRows', true), 1)
%                     fprintf("update\n");
%                 end
                % get a best path
                cameFrom{neighbour(1), neighbour(2), neighbour(3)} = [x y d idx];
                
                g(neighbour(1), neighbour(2), neighbour(3)) = score;
                f(neighbour(1), neighbour(2), neighbour(3)) = score + eps * heuristic(targetpos, neighbour);
            end
        end
    end
    
%     fprintf("cameFrom %d", cameFrom{fix(robotpos(1)/res+0.5), fix(robotpos(2)/res+0.5)}(3)); 
    fprintf("taking action %d\n", mprim_id);
%     [ret, motion] = applyaction(envmap, res, robotpos, mprim, dir, mprim_id);
%     newpos = motion(end,:);
% %     cameFrom{fix(newpos(1)/res+0.5), fix(newpos(2)/res+0.5)}
%     fprintf("ret %d\n", ret);
end
end

function [ h ] = heuristic(pos, target)
    targetang = target - pos;
    targetang = atan2(targetang(2), targetang(1));
    
    h = pdist([pos(1:2);target(1:2)], 'euclidean') + abs(targetang - pos(3)) * 100;
end
