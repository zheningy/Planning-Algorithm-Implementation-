/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include "mex.h"

/* Input Arguments */
#define	BLOCKSV_IN      prhs[0]
#define	TRIANGLESV_IN      prhs[1]
#define	TABLEINDEX_IN      prhs[2]
#define	ONVSTART_IN      prhs[3]
#define	CLEARVSTART_IN      prhs[4]
#define	ONVGOAL_IN      prhs[5]
#define	CLEARVGOAL_IN      prhs[6]
#define	MOVEACTIONINDEX_IN      prhs[7]
#define	MOVETOTABLEACTIONINDEX_IN      prhs[8]


/* Output Arguments */
#define	PLAN_OUT	plhs[0]

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

//blocksV - is an array of block indices (that is, if b is in blocks, then b
//is a block
//numofblocks - length of blocksV
//trianglesV - is an array of triangle indices
//numoftriangles - lengt of trianglesV
//TableIndex - index that corresponds to the table
//onV_start - a vector of On(x,y) statements that are true at start (each row is a statement, so
//OnV_start[0] says that item OnV_start[0][0] is on top of OnV_start[0][1]). Note: the 2nd dimension of OnV_start has 2 elements 
//onV_start_length - number of statements in OnV_start (that is, the size of the 1st dimension in OnV_start)
//clearV_start - is an array of items that are clear at start state (note Table is always clear
//by default)
//numofclear_start - length of clearV_start
//onV_goal - a vector of On(x,y) statements that are true at goal (same format as onV_start)
//onV_goal_length - number of statements in OnV_goal (that is, the size of the 1st dimension in OnV_goal)
//clearV_goal - is an array of items that are clear at goal state 
//numofclear_goal - length of clearV_goal
//moveActionIndex - index of the move(x,y,z) action that moves x from y to z
//(note that y could be a table but z should NOT be a table)
//moveToTableActionIndex - index of the moveToTable(x,y,z) action that moves x
//from y to z, where z is ALWAYS an index of the table
//plan - an array of action with their parameters. plan(i,:) is ith action.
//plan[i][0] - index of the action, where plan[i][1] - first argument, plan[i][2] - second argument, plan[i][3] - third argument 
static void planner(int* blocksV, int numofblocks, int* trianglesV, int numoftriangles, int TableIndex, 
            int** onV_start, int onV_start_length, int* clearV_start, int numofclear_start, 
            int** onV_goal, int onV_goal_length, int* clearV_goal, int numofclear_goal, 
            int moveActionIndex, int moveToTableActionIndex, int*** plan, int* planlength) 
{
	//no plan by default
	*plan = NULL;
	*planlength = 0;

    //this is where you insert your planner
    
    //allocate memory and set the return plan
    //for now this is an arbitrary sequence of actions (LIKELY INVALID)
    int numofsteps = 5;    
    *plan = (int**) malloc(numofsteps*sizeof(int*));
    int i;
    for (i = 0; i < numofsteps; i++){
        (*plan)[i] = (double*) malloc(4*sizeof(int)); 
        
        //just call move actions for even steps and movetotable actions for odd steps
        if(i%2 == 0)
        {
            //note this could be an invalid action since we are not checking if blocksV is clear and if it is on a table indeed
            (*plan)[i][0] = moveActionIndex;
            (*plan)[i][1] = blocksV[0];
            (*plan)[i][2] = TableIndex; 
            (*plan)[i][3] = blocksV[1];
            printf("%d %d %d %d\n", (*plan)[i][0], (*plan)[i][1], (*plan)[i][2], (*plan)[i][3]);
        }
        else
        {
            //note this could be an invalid action since we are not checking if blocksV is clear and if it is on a table indeed
            (*plan)[i][0] = moveToTableActionIndex;
            (*plan)[i][1] = blocksV[0];
            (*plan)[i][2] = blocksV[1]; 
            (*plan)[i][3] = TableIndex;                         
            printf("%d %d %d %d\n", (*plan)[i][0], (*plan)[i][1], (*plan)[i][2], (*plan)[i][3]);
        }
    }    
    *planlength = numofsteps;
    
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
    int *blocksV = malloc(sizeof(int)*numofblocks);
    for(i = 0; i < numofblocks; i++)
    {
        blocksV[i] = (int) blocksV_double[i];
        printf("block %d = %d\n", i, blocksV[i]);
    }
    
    /* get the triangles */
    double* trianglesV_double = mxGetPr(TRIANGLESV_IN);
    int numoftriangles = (int) (MAX(mxGetM(TRIANGLESV_IN), mxGetN(TRIANGLESV_IN)));
    int *trianglesV = malloc(sizeof(int)*numoftriangles);
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
    int **onV_start = malloc(sizeof(int*)*onV_start_length);
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
    int *clearstartV = malloc(sizeof(int)*numofclear_start);
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
    int **onV_goal = malloc(sizeof(int*)*onV_goal_length);
    for(i = 0; i < onV_goal_length; i++)
    {
        onV_goal[i] = malloc(sizeof(int)*2);
        onV_goal[i][0] = (int)onv_goal_double[0*onV_goal_length + i];
        onV_goal[i][1] = (int)onv_goal_double[1*onV_goal_length + i];
        printf("OnV at goal %d: %d is on %d\n", i, onV_goal[i][0], onV_goal[i][1]);
    }
    
    /*get the clearV for goal*/
    double* clearV_goal_double = mxGetPr(CLEARVGOAL_IN);
    int numofclear_goal = (int) (MAX(mxGetM(CLEARVGOAL_IN), mxGetN(CLEARVGOAL_IN)));
    int *cleargoalV = malloc(sizeof(int)*numofclear_goal);
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





