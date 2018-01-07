function [plan] = symbplanner(blocksV, trianglesV, TableIndex, onV_start, clearV_start, onV_goal, clearV_goal, moveActionIndex, moveToTableActionIndex);


%call the planner in C
plan = planner(blocksV, trianglesV, TableIndex, onV_start, clearV_start, onV_goal, clearV_goal, moveActionIndex, moveToTableActionIndex);

