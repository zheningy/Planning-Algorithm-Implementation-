/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include "mex.h"
#include <vector>
#include <set>
#include <map>

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
    State(){};
    State(int* blocksV, int numofblocks, int* trianglesV, int numoftriangles, int TableIndex,
          int** onV, int onV_length, int* clearV, int numofclear){
        vector<int> on_state;
        vector<int> clear_state;
        vector<int> shape_state (numofblocks + numoftriangles + 1);
        for (int i = 0; i < numofblocks + numoftriangles; i++)
        {
            bool no_on = true;
            bool no_clear = true;
            // load on state 
            for (int j = 0; j < onV_length; j++){
                if (onV[j][0] == i){
                    on_state.push_back(i);
                    on_state.push_back(onV[j][1]);
                    no_on = false;
                }
            }
            if (no_on){
                on_state.push_back(i);
                on_state.push_back(TableIndex);
            }
            // load clear state
            for (int j = 0; j < numofclear; j++){
                if (clearV[j] == i){
                    clear_state.push_back(1);
                    no_clear = false;
                }
            }
            if (no_clear){
                clear_state.push_back(0);
            }
            // load shape state
            for (int j = 0; j < numofblocks; j++){
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

    vector<vector<int>> state;
    int tableId;
};

bool moveToTable(State *state_out, int object_id, int from_id, int TableIndex, vector<vector<int>> &path){
    // pre condition
    if(state_out->state[0][2*object_id + 1] == from_id &&
       state_out->state[1][object_id] == 1 &&
       state_out->state[2][from_id] == 1){
        // effect
        //*state_out = state_in;
        state_out->state[0][2*object_id + 1] = state_out->tableId;
        state_out->state[1][from_id] = 1;
        vector<int> curr_path{1, object_id, from_id, TableIndex};
        path.push_back(curr_path);
        return true;
    }    
    else
        return false;
}

bool moveToBlock(State *state_out, int object_id, int from_id, int to_id, vector<vector<int>> &path){
    // pre condition
    if(state_out->state[0][2*object_id + 1] == from_id &&
       state_out->state[1][object_id] == 1 &&
       state_out->state[1][to_id] == 1 &&
       state_out->state[2][from_id] != 0 &&
       state_out->state[2][to_id] != 0 &&
       object_id != to_id){
        // effect
        //*state_out = state_in;
        state_out->state[0][2*object_id + 1] = to_id;
        state_out->state[1][from_id] = 1;
        state_out->state[1][to_id] = 0;
        vector<int> curr_path{0, object_id, from_id, to_id};
        path.push_back(curr_path);

        return true;
    }
    else{
        mexPrintf("state_ori: %d, from: %d, object_clear: %d, to_clear: %d, shape from, to: %d, %d\n",
            state_out->state[0][2*object_id + 1], 
            from_id, 
            state_out->state[1][object_id], 
            state_out->state[1][to_id],
            state_out->state[2][from_id],
            state_out->state[2][to_id]
            );
        return false;
    }
}

bool reachGoal(State now_state, vector<vector<int> > goal_state){
    for (int i = 0; i < goal_state[0].size()/2; i++){
        //printf("%d %d %d %d\n",i, goal_state[0][2*i + 1], now_state.state[0][2*goal_state[0][2*i]],2*goal_state[0][2*i]+1);
        if (goal_state[0][2*i + 1] != now_state.state[0][2*goal_state[0][2*i]+1])
            return false;
    }
    for (int j = 0; j < goal_state[1].size(); j++){
        if (now_state.state[1][goal_state[1][j]] != 1)
            return false; 
    }
    return true;
}

// move specific bolck to table, if could not make it, recursivly finish it.
bool mvTable(State *state_out, int object_id, int from_id, int TableIndex, vector<vector<int>> &path){
    while(state_out->state[1][object_id] != 1){
        for(int i = 0; i < state_out->state[0].size()/2; i++){
            if (state_out->state[0][2*i + 1] == object_id){
                mexPrintf("object %d need to be move to table\n", state_out->state[0][2*i]);
                mvTable(state_out, state_out->state[0][2*i], object_id,TableIndex, path);                
            }
        }
    }
    if (state_out->state[0][2*object_id + 1] != state_out->tableId){
        moveToTable(state_out, object_id, from_id, TableIndex, path);
        mexPrintf("object %d be moved to table\n", object_id);
    }
    else mexPrintf("object %d already on table\n", object_id);
}


static void planner(int* blocksV, int numofblocks, int* trianglesV, int numoftriangles, int TableIndex, 
            int** onV_start, int onV_start_length, int* clearV_start, int numofclear_start, 
            int** onV_goal, int onV_goal_length, int* clearV_goal, int numofclear_goal, 
            int moveActionIndex, int moveToTableActionIndex, int*** plan, int* planlength) 
{
    mexPrintf(" \n ");
    //no plan by default
    *plan = NULL;
    *planlength = 0;

    int numofsteps = 0;
    vector<vector<int>> path;

    State a = State(blocksV, numofblocks, trianglesV, numoftriangles, TableIndex, 
            onV_start, onV_start_length, clearV_start, numofclear_start);

    // save all linked blocks at goal state
    vector<vector<int> *> linkBlocks;
    // save all blocks need to be move to table(in on and clear state)
    set<int> related_set;
    // Hashmap save on relationship at goal state
    map<int, int> onStateMap;
    for(int i = 0; i < onV_goal_length; i++){
        related_set.insert(onV_goal[i][0]);
        related_set.insert(onV_goal[i][1]);
        onStateMap.insert({onV_goal[i][1],onV_goal[i][0]});
        bool inital = true;
        for (int j =0; j < onV_goal_length; j++){
            if(onV_goal[i][1] == onV_goal[j][0]){
                inital = false;
                break;
            }
        }
        if (inital){
            linkBlocks.push_back(new vector<int>);
            int row = linkBlocks.size();
            linkBlocks.at(row - 1)->push_back(onV_goal[i][1]);
        }
    }

    for(int i = 0; i < numofclear_goal; i++){
        related_set.insert(clearV_goal[i]);
    }

    // for ( auto it = onStateMap.begin(); it != onStateMap.end(); ++it )
    //     mexPrintf("key : %d value %d\n",  it->first, it->second);


    // Construct all listed blocks    
    for(int i = 0; i < linkBlocks.size(); i++){
        int father_node = linkBlocks[i]->at(0);
        auto tem_val = onStateMap.find(father_node);
        while(tem_val != onStateMap.end()){
            father_node = tem_val->second;
            linkBlocks[i]->push_back(father_node);
            tem_val = onStateMap.find(father_node);
        }
    }

    // Move all related bolcks to table
    mexPrintf("initial point : %d \n", linkBlocks[0]->at(0));
    for (auto it = related_set.begin(); it != related_set.end(); ++it){
        int cand_id = *it;
        
        if (a.state[0][cand_id*2 + 1] != TableIndex || a.state[1][cand_id] == 0){
            mexPrintf("candiate id: %d \n", cand_id);
            mvTable(&a, a.state[0][cand_id*2], a.state[0][cand_id*2 + 1], TableIndex, path);
        }
    }


    // move blocks to goal state
    for(int i = 0; i < linkBlocks.size(); i++){
        mexPrintf("linkBlocks size %d\n", linkBlocks[i]->size());
        for(int j = 0; j < linkBlocks[i]->size() - 1; j++){
            int target_node = linkBlocks[i]->at(j);
            int moving_node = linkBlocks[i]->at(j + 1);
            if(moveToBlock(&a, moving_node, TableIndex, target_node, path))
                mexPrintf("object %d be moved from table to %d\n", moving_node, target_node);
            else
                mexPrintf("Fail to move object %d from table to %d\n", moving_node, target_node);
        }
    }

    mexPrintf("final path %d", path.size());


    // Record path to matlab
    numofsteps = path.size();
    *planlength = numofsteps;
    *plan = (int**) malloc(numofsteps*sizeof(int*));

    for (int i = 0; i < numofsteps; i++){
        (*plan)[i] = (int*) malloc(4*sizeof(int));
        (*plan)[i][0] = path[i][0];
        (*plan)[i][1] = path[i][1];
        (*plan)[i][2] = path[i][2]; 
        (*plan)[i][3] = path[i][3]; 
    }



     mexPrintf(" \n ");    
    return;
}



//prhs contains input parameters (9): 
//1st is blocksV - is an array of block indices (that is, if b is in blocks, then b
//is a block
//2nd is trianglesV - is an array of triangle indices
//3rd is TableIndex - index that corresponds to the table
//4th is onV_start - a vector of On(x,y) statements that are true at start (each row is a statement, so
//OnV[0] says that item OnV[0][0] is on top of OnV[0][1]
//5th is clearV_start - is an array of items that are clear at start state (note Table is always clear
//by default)
//6th is onV_goal - a vector of On(x,y) statements that are true at goal 
//7th i clearV_goal - is an array of items that are clear at goal state 
//8th is moveActionIndex - index of the move(x,y,z) action that moves x from y to z
//(note that y could be a table but z should NOT be a table)
//9th is moveToTableActionIndex - index of the moveToTable(x,y,z) action that moves x
//from y to z, where z is ALWAYS an index of the table

//plhs should contain output parameters (1): 
//plan - an array of action with their parameters. plan(i,:) is ith action.
//plan[i][0] - index of the action, where plan[i][1] - first argument, plan[i][2] - second argument, plan[i][3] - third argument 
void mexFunction( int nlhs, mxArray *plhs[], 
          int nrhs, const mxArray*prhs[])
     
{ 
    
    /* Check for proper number of arguments */    
    if (nrhs != 9) { 
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Nine input arguments required."); 
    } else if (nlhs != 1) {
        mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required."); 
    } 

    int i = 0;

    /* get the blocks */
    double* blocksV_double = mxGetPr(BLOCKSV_IN);
    int numofblocks = (int) (MAX(mxGetM(BLOCKSV_IN), mxGetN(BLOCKSV_IN)));
    if(numofblocks < 2)
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumofBlocks",
                "At least two blocks are required.");         
    int *blocksV = (int *)malloc(sizeof(int)*numofblocks);
    for(i = 0; i < numofblocks; i++)
    {
        blocksV[i] = (int) blocksV_double[i];
        printf("block %d = %d\n", i, blocksV[i]);
    }
    
    /* get the triangles */
    double* trianglesV_double = mxGetPr(TRIANGLESV_IN);
    int numoftriangles = (int) (MAX(mxGetM(TRIANGLESV_IN), mxGetN(TRIANGLESV_IN)));
    int *trianglesV = (int *)malloc(sizeof(int)*numoftriangles);
    for(i = 0; i < numoftriangles; i++)
    {
        trianglesV[i] = (int) trianglesV_double[i];
        printf("triangle %d = %d\n", i, trianglesV[i]);
    }

    /*get the table index */
    int TableIndex = (int)(*mxGetPr(TABLEINDEX_IN));
    printf("TableIndex=%d\n", TableIndex);
    
    /*get the onV for start*/
    int onV_start_length = (int) mxGetM(ONVSTART_IN);
    int onV_start_cols = (int) mxGetN(ONVSTART_IN);
    double* onv_start_double = mxGetPr(ONVSTART_IN);
    if(onV_start_cols != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidonv_start",
                "each onv_start statement should have 3 parameters");         
    }
    int **onV_start = (int **)malloc(sizeof(int*)*onV_start_length);
    for(i = 0; i < onV_start_length; i++)
    {
        onV_start[i] = (int*)(malloc(sizeof(int)*2));
        onV_start[i][0] = (int)onv_start_double[0*onV_start_length + i];
        onV_start[i][1] = (int)onv_start_double[1*onV_start_length + i];
        printf("OnV at start %d: %d is on %d\n", i, onV_start[i][0], onV_start[i][1]);
    }
        
    /*get the clearV for start*/
    double* clearV_start_double = mxGetPr(CLEARVSTART_IN);
    int numofclear_start = (int) (MAX(mxGetM(CLEARVSTART_IN), mxGetN(CLEARVSTART_IN)));
    int *clearstartV = (int *)malloc(sizeof(int)*numofclear_start);
    for(i = 0; i < numofclear_start; i++)
    {
        clearstartV[i] = (int) clearV_start_double[i];
        printf("clear at start %d: %d is clear\n", i, clearstartV[i]);
    }

    
    /*get the onV for goal*/
    int onV_goal_length = (int) mxGetM(ONVGOAL_IN);
    int onV_goal_cols = (int) mxGetN(ONVGOAL_IN);
    double* onv_goal_double = mxGetPr(ONVGOAL_IN);
    if(onV_goal_cols != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidonv_goal",
                "each onv_goal statement should have 3 parameters");         
    }
    int **onV_goal = (int **)malloc(sizeof(int*)*onV_goal_length);
    for(i = 0; i < onV_goal_length; i++)
    {
        onV_goal[i] = (int *)malloc(sizeof(int)*2);
        onV_goal[i][0] = (int)onv_goal_double[0*onV_goal_length + i];
        onV_goal[i][1] = (int)onv_goal_double[1*onV_goal_length + i];
        printf("OnV at goal %d: %d is on %d\n", i, onV_goal[i][0], onV_goal[i][1]);
    }
    
    /*get the clearV for goal*/
    double* clearV_goal_double = mxGetPr(CLEARVGOAL_IN);
    int numofclear_goal = (int) (MAX(mxGetM(CLEARVGOAL_IN), mxGetN(CLEARVGOAL_IN)));
    int *cleargoalV = (int *)malloc(sizeof(int)*numofclear_goal);
    for(i = 0; i < numofclear_goal; i++)
    {
        cleargoalV[i] = (int) clearV_goal_double[i];
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
    if(planlength > 0)
    {
        PLAN_OUT = mxCreateNumericMatrix( (mwSize)planlength, (mwSize)4, mxDOUBLE_CLASS, mxREAL); 
        double* plan_out = mxGetPr(PLAN_OUT);        
        //copy the values
        int i,j;
        for(i = 0; i < planlength; i++)
        {
            for (j = 0; j < 4; j++)
            {
                plan_out[j*planlength + i] = (double)plan[i][j];
            }
        }
    }
    else
    {
        PLAN_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)1, mxDOUBLE_CLASS, mxREAL); 
        double* plan_out = mxGetPr(PLAN_OUT);
        *plan_out = 0;
    }
            
            
    return;
    
}





