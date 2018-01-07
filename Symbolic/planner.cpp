/*=================================================================
*
* planner.c
*
*=================================================================*/
#include <math.h>
#include "mex.h"
#include <vector>
#include <algorithm>
#include <set>
#include <map>
#include <time.h>
#include <memory>
#include <stdlib.h>

/* Input Arguments */
#define BLOCKSV_IN      prhs[0]
#define TRIANGLESV_IN      prhs[1]
#define TABLEINDEX_IN      prhs[2]
#define ONVSTART_IN      prhs[3]
#define CLEARVSTART_IN      prhs[4]
#define ONVGOAL_IN      prhs[5]
#define CLEARVGOAL_IN      prhs[6]
#define MOVEACTIONINDEX_IN      prhs[7]
#define MOVETOTABLEACTIONINDEX_IN      prhs[8]


/* Output Arguments */
#define PLAN_OUT    plhs[0]

#if !defined(MAX)
#define MAX(A, B)   ((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define MIN(A, B)   ((A) < (B) ? (A) : (B))
#endif

using namespace std;
class State
{
public:
	vector<vector<int>> state;          // Store position relationship, clear state, shape of each blocks.
	vector<weak_ptr<State>> neighbors;  // Store points to all successors state
	weak_ptr<State> before;             // Point to the father state of current state.
	vector<int> pre_path;               // Store the movement from father state to current state.    
	int tableId;
	int cost;

	State() {};
	State(int c): cost(c) {};
	State(int c, int* blocksV, int numofblocks, int* trianglesV, int numoftriangles, int TableIndex,
		int** onV, int onV_length, int* clearV, int numofclear) : cost(c) {

		vector<int> on_state;
		vector<int> clear_state;
		vector<int> shape_state(numofblocks + numoftriangles + 1);
		for (int i = 0; i < numofblocks + numoftriangles; i++)
		{
			bool no_on = true;
			bool no_clear = true;
			// load on state 
			for (int j = 0; j < onV_length; j++) {
				if (onV[j][0] == i) {
					on_state.push_back(i);
					on_state.push_back(onV[j][1]);
					no_on = false;
				}
			}
			if (no_on) {
				on_state.push_back(i);
				on_state.push_back(TableIndex);
			}
			// load clear state
			for (int j = 0; j < numofclear; j++) {
				if (clearV[j] == i) {
					clear_state.push_back(1);
					no_clear = false;
				}
			}
			if (no_clear) {
				clear_state.push_back(0);
			}
			// load shape state
			for (int j = 0; j < numofblocks; j++) {
				if (blocksV[j] == i)
					shape_state[i] = 1;
			}
			shape_state[TableIndex] = 2;
		}
		this->state.push_back(on_state);
		this->state.push_back(clear_state);
		this->state.push_back(shape_state);
		this->tableId = TableIndex;
	}
	
	// Check if two states are identical
	bool isequal(const shared_ptr<State> &other_state) {
		if (other_state->state[0] == this->state[0] &&
			other_state->state[1] == this->state[1]) return true;
		else return false;
	}
};


bool moveToTable(const shared_ptr<State> &state_in, shared_ptr<State> &state_out, int object_id, int from_id, int TableIndex) {
	// pre condition
	if (state_in->state[0][2 * object_id + 1] == from_id &&
		state_in->state[1][object_id] == 1 &&
		state_in->state[2][from_id] != 0) {
		// effect
		*state_out = *state_in;
		state_out->state[0][2 * object_id + 1] = state_out->tableId;
		state_out->state[1][from_id] = 1;
		state_out->cost = 1000000;
		return true;
	}
	else
		return false;
}

bool moveToBlock(const shared_ptr<State> &state_in, shared_ptr<State> &state_out, int object_id, int from_id, int to_id) {
	// pre condition
	if (state_in->state[0][2 * object_id + 1] == from_id &&
		state_in->state[1][object_id] == 1 &&
		state_in->state[1][to_id] == 1 &&
		state_in->state[2][from_id] != 0 &&
		state_in->state[2][to_id] != 0 &&
		object_id != to_id) {
		// effect
		*state_out = *state_in;
		state_out->state[0][2 * object_id + 1] = to_id;
		state_out->state[1][from_id] = 1;
		state_out->state[1][to_id] = 0;
		state_out->cost = 1000000;
		return true;
	}
	else return false;
}


// Find all moveable actions for state
vector<vector<int>> findMoves(const shared_ptr<State> &state_in, int TableIndex) {
	vector<vector<int>> result;
	vector<int> cand_targets;
	for (int i = 0; i < state_in->state[1].size(); i++) {
		if (state_in->state[1][i] != 0) cand_targets.push_back(i);
	}
	for (int i = 0; i < cand_targets.size(); i++) {
		int moveId = cand_targets[i];
		if (state_in->state[0][2 * moveId + 1] != TableIndex) {
			vector<int> table_move = vector<int>{ 1, moveId, state_in->state[0][2 * moveId + 1], TableIndex };
			result.push_back(table_move);
		}
		for (int j = 0; j < cand_targets.size(); j++) {
			if (i == j || state_in->state[2][cand_targets[j]] == 0) continue;
			else {
				int goalId = cand_targets[j];
				vector<int> block_move = vector<int>{ 0, moveId, state_in->state[0][2 * moveId + 1], goalId };
				result.push_back(block_move);
			}
		}
	}
	return result;
}

// Check if current state satisify goal state.
bool reachGoal(const shared_ptr<State> &now_state, const vector<vector<int>> &goal_state) {
	for (int i = 0; i < goal_state[0].size() / 2; i++) {
		if (goal_state[0][2 * i + 1] != now_state->state[0][2 * goal_state[0][2 * i] + 1])
			return false;
	}
	for (int j = 0; j < goal_state[1].size(); j++) {
		if (now_state->state[1][goal_state[1][j]] != 1)
			return false;
	}
	mexPrintf("Reach Goal! \n");
	return true;
}

// Calculate the distance(the number of different position relationship between current state and goal state)
int disToGoal(const shared_ptr<State> &now_state, const vector<vector<int>> &goal_state) {
	int dis = now_state->state[1].size();

	for (int i = 0; i < goal_state[0].size() / 2; i++) {
		if (goal_state[0][2 * i + 1] == now_state->state[0][2 * goal_state[0][2 * i] + 1])
			dis -= 1;
	}
	for (int j = 0; j < goal_state[1].size(); j++) {
		if (now_state->state[1][goal_state[1][j]] == 1)
			dis -= 1;
	}

	return(max(1, dis));

}


static void planner(int* blocksV, int numofblocks, int* trianglesV, int numoftriangles, int TableIndex,
	int** onV_start, int onV_start_length, int* clearV_start, int numofclear_start,
	int** onV_goal, int onV_goal_length, int* clearV_goal, int numofclear_goal,
	int moveActionIndex, int moveToTableActionIndex, int*** plan, int* planlength)
{
	//no plan by default
	*plan = NULL;
	*planlength = 0;

	// Record goal incomplete state
	vector<vector<int> > goal_state;
	vector<int> goal_on_state;
	vector<int> goal_clear_state;
	for (int i = 0; i < onV_goal_length; i++) {
		goal_on_state.push_back(onV_goal[i][0]);
		goal_on_state.push_back(onV_goal[i][1]);
	}
	for (int i = 0; i < numofclear_goal; i++) {
		goal_clear_state.push_back(clearV_goal[i]);
	}
	goal_state.push_back(goal_on_state);
	goal_state.push_back(goal_clear_state);
		
	vector<shared_ptr<State>> states;   // Nodes save all state

	auto node_start = make_shared<State>(0, blocksV, numofblocks, trianglesV, numoftriangles, TableIndex,
		onV_start, onV_start_length, clearV_start, numofclear_start);

	states.push_back(node_start);

	int j = 0;
	// Create priority set as closed set and open set
	auto cmp = [](const shared_ptr<State> &n1, const shared_ptr<State> &n2) { return n1->cost <= n2->cost; };
	set<shared_ptr<State>, decltype(cmp)> open_set(cmp);
	set<shared_ptr<State>, decltype(cmp)> closed_set(cmp);

	// A*
	open_set.insert(node_start);
	auto next = *open_set.begin();
	open_set.erase(open_set.begin());
	while (reachGoal(next, goal_state) == false && j++ < 20000) {
		auto cand_moves = findMoves(next, TableIndex);
		for (const auto move : cand_moves) {
			auto tem_node = make_shared<State>(100000);
			if (move[0] == 1) moveToTable(next, tem_node, move[1], move[2], move[3]);
			else moveToBlock(next, tem_node, move[1], move[2], move[3]);
			bool unique = true;         // Flag used to check whether the new expanded state not appear before.
			for (auto &exist_state : states) {
				if (exist_state->isequal(tem_node)) {
					unique = false;
					break;
				}
			}
			if (unique) {
				tem_node->pre_path = move;
				//tem_node->neighbors.push_back(next);
				next->neighbors.push_back(tem_node);
				states.push_back(tem_node);
			}
		}
		
		closed_set.insert(next);
		for (const auto n : next->neighbors) {
			auto p = n.lock();		
			if (closed_set.find(p) == closed_set.end()) {
				double g_cand = next->cost + disToGoal(p, goal_state);  // heuristic function.
				//double g_cand = next->cost + 1;
				if (g_cand < p->cost) {
					p->before = next;
					p->cost = g_cand;
					open_set.insert(p);
				}
			}
		}
		next = *open_set.begin();
		open_set.erase(open_set.begin());
		
		//mexPrintf("now state : %d %d, %d %d, %d %d, %d %d, %d %d, %d %d, %d %d \n", next->state[0][0], next->state[0][1],
		//	next->state[0][2], next->state[0][3], next->state[0][4], next->state[0][5], next->state[0][6], next->state[0][7],
		//	next->state[0][8], next->state[0][9], next->state[0][10], next->state[0][11], next->state[0][12], next->state[0][13]);
		int num = cand_moves.size();
		//mexPrintf("move options %d \n", num);
	}

	// Record plan for display in matlab
	mexPrintf("State expand %d \n", j);
	int numofsteps = 0;
	vector<vector<int>> path;
	auto iter = next;
	while (iter) {
		path.push_back(iter->pre_path);
		iter = iter->before.lock();
	}
	reverse(path.begin(), path.end());
	numofsteps = path.size()-1;

	*planlength = numofsteps;
	*plan = (int**)malloc(numofsteps * sizeof(int*));

	for (int i = 1; i < path.size() ; i++) {
		(*plan)[i-1] = (int*)malloc(4 * sizeof(int));
		(*plan)[i-1][0] = path[i][0];
		(*plan)[i-1][1] = path[i][1];
		(*plan)[i-1][2] = path[i][2];
		(*plan)[i-1][3] = path[i][3];
	}

	mexPrintf("final path %d\n", path.size()-1);
	return;
}



void mexFunction(int nlhs, mxArray *plhs[],
	int nrhs, const mxArray*prhs[])

{

	/* Check for proper number of arguments */
	if (nrhs != 9) {
		mexErrMsgIdAndTxt("MATLAB:planner:invalidNumInputs",
			"Nine input arguments required.");
	}
	else if (nlhs != 1) {
		mexErrMsgIdAndTxt("MATLAB:planner:maxlhs",
			"One output argument required.");
	}

	int i = 0;

	/* get the blocks */
	double* blocksV_double = mxGetPr(BLOCKSV_IN);
	int numofblocks = (int)(MAX(mxGetM(BLOCKSV_IN), mxGetN(BLOCKSV_IN)));
	if (numofblocks < 2)
		mexErrMsgIdAndTxt("MATLAB:planner:invalidNumofBlocks",
			"At least two blocks are required.");
	int *blocksV = (int *)malloc(sizeof(int)*numofblocks);
	for (i = 0; i < numofblocks; i++)
	{
		blocksV[i] = (int)blocksV_double[i];
		printf("block %d = %d\n", i, blocksV[i]);
	}

	/* get the triangles */
	double* trianglesV_double = mxGetPr(TRIANGLESV_IN);
	int numoftriangles = (int)(MAX(mxGetM(TRIANGLESV_IN), mxGetN(TRIANGLESV_IN)));
	int *trianglesV = (int *)malloc(sizeof(int)*numoftriangles);
	for (i = 0; i < numoftriangles; i++)
	{
		trianglesV[i] = (int)trianglesV_double[i];
		printf("triangle %d = %d\n", i, trianglesV[i]);
	}

	/*get the table index */
	int TableIndex = (int)(*mxGetPr(TABLEINDEX_IN));
	printf("TableIndex=%d\n", TableIndex);

	/*get the onV for start*/
	int onV_start_length = (int)mxGetM(ONVSTART_IN);
	int onV_start_cols = (int)mxGetN(ONVSTART_IN);
	double* onv_start_double = mxGetPr(ONVSTART_IN);
	if (onV_start_cols != 2) {
		mexErrMsgIdAndTxt("MATLAB:planner:invalidonv_start",
			"each onv_start statement should have 3 parameters");
	}
	int **onV_start = (int **)malloc(sizeof(int*)*onV_start_length);
	for (i = 0; i < onV_start_length; i++)
	{
		onV_start[i] = (int*)(malloc(sizeof(int) * 2));
		onV_start[i][0] = (int)onv_start_double[0 * onV_start_length + i];
		onV_start[i][1] = (int)onv_start_double[1 * onV_start_length + i];
		printf("OnV at start %d: %d is on %d\n", i, onV_start[i][0], onV_start[i][1]);
	}

	/*get the clearV for start*/
	double* clearV_start_double = mxGetPr(CLEARVSTART_IN);
	int numofclear_start = (int)(MAX(mxGetM(CLEARVSTART_IN), mxGetN(CLEARVSTART_IN)));
	int *clearstartV = (int *)malloc(sizeof(int)*numofclear_start);
	for (i = 0; i < numofclear_start; i++)
	{
		clearstartV[i] = (int)clearV_start_double[i];
		printf("clear at start %d: %d is clear\n", i, clearstartV[i]);
	}


	/*get the onV for goal*/
	int onV_goal_length = (int)mxGetM(ONVGOAL_IN);
	int onV_goal_cols = (int)mxGetN(ONVGOAL_IN);
	double* onv_goal_double = mxGetPr(ONVGOAL_IN);
	if (onV_goal_cols != 2) {
		mexErrMsgIdAndTxt("MATLAB:planner:invalidonv_goal",
			"each onv_goal statement should have 3 parameters");
	}
	int **onV_goal = (int **)malloc(sizeof(int*)*onV_goal_length);
	for (i = 0; i < onV_goal_length; i++)
	{
		onV_goal[i] = (int *)malloc(sizeof(int) * 2);
		onV_goal[i][0] = (int)onv_goal_double[0 * onV_goal_length + i];
		onV_goal[i][1] = (int)onv_goal_double[1 * onV_goal_length + i];
		printf("OnV at goal %d: %d is on %d\n", i, onV_goal[i][0], onV_goal[i][1]);
	}

	/*get the clearV for goal*/
	double* clearV_goal_double = mxGetPr(CLEARVGOAL_IN);
	int numofclear_goal = (int)(MAX(mxGetM(CLEARVGOAL_IN), mxGetN(CLEARVGOAL_IN)));
	int *cleargoalV = (int *)malloc(sizeof(int)*numofclear_goal);
	for (i = 0; i < numofclear_goal; i++)
	{
		cleargoalV[i] = (int)clearV_goal_double[i];
		printf("clear at goal %d: %d is clear\n", i, cleargoalV[i]);
	}

	/*get the moveAction index */
	int moveActionIndex = (int)(*mxGetPr(MOVEACTIONINDEX_IN));
	printf("moveActionIndex=%d\n", moveActionIndex);

	/*get the moveToTableAction index */
	int moveToTableActionIndex = (int)(*mxGetPr(MOVETOTABLEACTIONINDEX_IN));
	printf("moveToTableActionIndex=%d\n", moveToTableActionIndex);

	//call the planner
	int** plan = NULL;
	int planlength = 0;

	planner(blocksV, numofblocks, trianglesV, numoftriangles, TableIndex,
		onV_start, onV_start_length, clearstartV, numofclear_start, onV_goal, onV_goal_length, cleargoalV, numofclear_goal,
		moveActionIndex, moveToTableActionIndex, &plan, &planlength);

	printf("planner returned plan of length=%d\n", planlength);

	/* Create return values */
	if (planlength > 0)
	{
		PLAN_OUT = mxCreateNumericMatrix((mwSize)planlength, (mwSize)4, mxDOUBLE_CLASS, mxREAL);
		double* plan_out = mxGetPr(PLAN_OUT);
		//copy the values
		int i, j;
		for (i = 0; i < planlength; i++)
		{
			for (j = 0; j < 4; j++)
			{
				plan_out[j*planlength + i] = (double)plan[i][j];
			}
		}
	}
	else
	{
		PLAN_OUT = mxCreateNumericMatrix((mwSize)1, (mwSize)1, mxDOUBLE_CLASS, mxREAL);
		double* plan_out = mxGetPr(PLAN_OUT);
		*plan_out = 0;
	}


	return;

}





