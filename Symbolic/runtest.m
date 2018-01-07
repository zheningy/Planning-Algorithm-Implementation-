function[plan] = runtest(blocksV, trianglesV, TableIndex, onV_start, clearV_start, onV_goal, clearV_goal, moveActionIndex, moveToTableActionIndex)

%blocksV - is an array of block indices (that is, if b is in blocks, then b
%is a block

% trianglesV - is an array of triangle indices

% TableIndex - index that corresponds to the table

% onV_start - a vector of On(x,y) statements that are true at start (each row is a statement, so
% OnV(1,:) says that item OnV(1,1) is on top of OnV(1,2)

% clearV_start - is an array of items that are clear at start state (note Table is always clear
% by default)

% onV_goal - a vector of On(x,y) statements that are true at goal 

% clearV_goal - is an array of items that are clear at goal state 

% moveActionIndex - index of the move(x,y,z) action that moves x from y to z
%(note that y could be a table but z should NOT be a table)

% moveToTableActionIndex - index of the moveToTable(x,y,z) action that moves x
%from y to z, where z is ALWAYS an index of the table

% plan - an array of action with their parameters. plan(i,:) is ith action.
% plan(i,1) - index of the action, where plan(i,2) - first argument, plan(i,3) - second argument, plan(i,4) - third argument 

close all;

%call the planner
plan = symbplanner(blocksV, trianglesV, TableIndex, onV_start, clearV_start, onV_goal, clearV_goal, moveActionIndex, moveToTableActionIndex); 

fprintf(1, 'plan of length %d was found\n', size(plan,1));

%print the plan. NOTE: the check for action validity is ONLY PARTIAL. It is
%your responsibility to ensure that all the actions in the plan are valid.
for i = 1:size(plan,1)
    movetotable = 0;
    if(plan(i,1) == moveActionIndex)
        fprintf(1, 'move ');
    elseif(plan(i,1) == moveToTableActionIndex)
        fprintf(1, 'moveToTable ');
        movetotable = 1;
    else
        fprintf(1, 'ERROR: nonexistent action %d\n', plan(i,1));
        break;
    end;
    
    if(ismember(plan(i,2), blocksV))
        fprintf(1, 'block %d from ', plan(i,2));
    elseif(ismember(plan(i,2), trianglesV))
        fprintf(1, 'triangle %d from ', plan(i,2));
    else
        fprintf(1, 'ERROR: invalid parameter %d\n', plan(i,2));
        break;
    end;

    if(ismember(plan(i,3), blocksV))
        fprintf(1, 'block %d to ', plan(i,3))
    elseif(plan(i,3) == TableIndex)
        fprintf(1, 'table to ');
    else
        fprintf(1, 'ERROR: invalid parameter %d\n', plan(i,3));
        break;
    end;
    
    if(ismember(plan(i,4), blocksV) && movetotable == 0)
        fprintf(1, 'block %d\n', plan(i,4))
    elseif(plan(i,4) == TableIndex && movetotable == 1)
        fprintf(1, 'table\n')
    else
        fprintf(1, 'ERROR: invalid parameter %d\n', plan(i,4));
        break;
    end;    
end;

%armplan