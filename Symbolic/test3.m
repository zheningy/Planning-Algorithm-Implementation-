blocksV = [0 1 3 5 6 7 8];
trianglesV = [2 4];
TableIndex = 9;
onV_start = [0 1; 2 0; 5 3; 6 5; 4 6;1 7];
clearV_start = [2 4 8];
onV_goal = [2 6; 6 0; 0 3; 4 1; 5 8];
clearV_goal = [2,4];
moveActionIndex = 0;
moveToTableActionIndex = 1;
runtest(blocksV, trianglesV, TableIndex, onV_start, clearV_start, onV_goal, clearV_goal, moveActionIndex, moveToTableActionIndex);
