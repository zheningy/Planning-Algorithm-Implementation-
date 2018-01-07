/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include "mex.h"
#include <stdio.h>
#include <stdlib.h>
#include "memory.h"
#include <string.h>
#include <vector>
#include <algorithm>
using namespace std;

/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ROBOT_IN	prhs[1]
#define	GOAL_IN     prhs[2]


/* Output Arguments */
#define	ACTION_OUT	plhs[0]

/*access to the map is shifted to account for 0-based indexing in the map, whereas
1-based indexing in matlab (so, robotpose and goalpose are 1-indexed)*/
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

/* Primitives Information */
#define NUMOFDIRS 8
#define NUMOFPRIMS 5
#define NUMOFINTERSTATES 10
#define NUMOFDIM 3

#define RES 0.1

# define M_PI           3.14159265358979323846 
class State
{
public:
    State() {};
    State(float x, float y, float theta, float gh) {
        this->x = x;
        this->y = y;
        this->theta = theta;
        this->gh = gh;
    };

    float x, y, theta, gh;
};

class Pred {
    
public:
    Pred(float x, float y, float theta, int prim) {
        this->x = x;
        this->y = y;
        this->theta = theta;
        this->prim_before = prim;
    }
    float x, y, theta;
    int prim_before;
};

float heuristic(float robotposeX,
                float robotposeY,
                float robotposeTheta,
                float goalposeX,
                float goalposeY) {
    float x_diff = robotposeX - goalposeX;
    float y_diff = robotposeY - goalposeY;
    return sqrtf(x_diff * x_diff + y_diff * y_diff);
}

bool closeTo(float x1, float y1, float x2, float y2) {
    float eps = 1e-4f;
    if (fabsf(x1 - x2) < eps && fabsf(y1 - y2) < eps) return true;
    return false;
}


typedef float PrimArray[NUMOFDIRS][NUMOFPRIMS][NUMOFINTERSTATES][NUMOFDIM];

int temp = 0;

bool applyaction(double *map, int x_size, int y_size, float robotposeX, float robotposeY, float robotposeTheta,
                 float *newx, float *newy, float *newtheta, PrimArray mprim, int dir, int prim)
{
    int i;
    for (i = 0; i < NUMOFINTERSTATES; i++) {
        *newx = robotposeX + mprim[dir][prim][i][0];
        *newy = robotposeY + mprim[dir][prim][i][1];
        *newtheta = mprim[dir][prim][i][2];
        
        int gridposx = (int)roundf(*newx / RES);
        int gridposy = (int)roundf(*newy / RES);

        /* check validity */
        if (gridposx <= 1 || gridposx >= x_size || gridposy <= 1 || gridposy >= y_size){
            return false;
        }
        if ((int)map[GETMAPINDEX(gridposx, gridposy, x_size, y_size)] != 0){
            return false;
        }
    }


    return true;
}

int getPrimitiveDirectionforRobotPose(float angle)
{
    /* returns the direction index with respect to the PrimArray */
    /* normalize bw 0 to 2pi */
    if (angle < 0.0) {
        angle += 2 * M_PI;
    }
    int dir = (int)roundf(angle / (2 * M_PI / NUMOFDIRS));
    if (dir == 8) {
        dir = 0;
    }
    return dir;
}

static void forwardAstar(
            double*   map,
            int x_size,
            int y_size,
            float robotposeX,
            float robotposeY,
            float robotposeTheta,
            float goalposeX,
            float goalposeY,
            PrimArray mprim,
            int *prim_id)
{
    float dist = sqrtf((robotposeX - goalposeX) * (robotposeX - goalposeX) + (robotposeY - goalposeY) * (robotposeY - goalposeY));
    float eps = dist > 10 ? dist : 1.f;
    vector<vector<float>> g(x_size, vector<float>(y_size,  1000000.f));
    vector<vector<bool>> open_set(x_size, vector<bool>(y_size, false));
    vector<vector<bool>> closed_set(x_size, vector<bool>(y_size, false));
    vector<vector<Pred>> prim_from(x_size, vector<Pred>(y_size, Pred(0, 0, 0, -1)));
    g[(int)roundf(robotposeX / RES)][(int)roundf(robotposeY / RES)] = 0.f;
    State s_init(robotposeX, robotposeY, robotposeTheta, eps * heuristic(robotposeX, robotposeY, robotposeTheta, goalposeX, goalposeY));
    vector<State> states;
    states.push_back(s_init);
    
    auto next_p = min_element(states.begin(), states.end(), [](const State& a, const State& b) {
        return a.gh < b.gh;
    });
    auto next = *next_p;
    states.erase(next_p);
    while (!closeTo(next.x, next.y, goalposeX, goalposeY)) {
        int dir = getPrimitiveDirectionforRobotPose(next.theta);
        open_set[(int)roundf(next.x / RES)][(int)roundf(next.y / RES)] = false;
        closed_set[(int)roundf(next.x / RES)][(int)roundf(next.y / RES)] = true;
        for (int prim = 0; prim < NUMOFPRIMS; prim++) {
            float newx, newy, newtheta;
            bool ret = applyaction(map, x_size, y_size, next.x, next.y, next.theta, &newx, &newy, &newtheta, mprim, dir, prim);
            /* skip action that leads to collision */
            if (ret && !closed_set[(int)roundf(newx / RES)][(int)roundf(newy / RES)]) {
                float g_cand = g[(int)roundf(next.x / RES)][(int)roundf(next.y / RES)]
                + sqrtf((newx - next.x) * (newx - next.x) + (newy - next.y) * (newy - next.y));
                if (g_cand < g[(int)roundf(newx / RES)][(int)roundf(newy / RES)]) {
                    g[(int)roundf(newx / RES)][(int)roundf(newy / RES)] = g_cand;
                    // printf("%f\n", g_cand);
                    if (!open_set[(int)roundf(newx / RES)][(int)roundf(newy / RES)]) {
                        State s(newx, newy, newtheta, g_cand + eps * heuristic(newx, newy, newtheta, goalposeX, goalposeY));
                        open_set[(int)roundf(newx / RES)][(int)roundf(newy / RES)] = true;
                        prim_from[(int)roundf(newx / RES)][(int)roundf(newy / RES)] = Pred(next.x, next.y, next.theta, prim);
                        states.push_back(s);
                    }
                }
            }
        }
        if (states.empty()) break;
        next_p = min_element(states.begin(), states.end(), [](const State& a, const State& b) {
            return a.gh < b.gh;
        });
        next = *next_p;
        states.erase(next_p);
    }
    
    double mindisttotarget = 1000000;
    *prim_id = 0;
    Pred pred = prim_from[(int)roundf(next.x / RES)][(int)roundf(next.y / RES)];
    if (pred.prim_before == -1) {
        for (int prim = 0; prim < NUMOFPRIMS; prim++) {
            float newx, newy, newtheta;
            bool ret;
            int dir;
            dir = getPrimitiveDirectionforRobotPose(next.theta);
            ret = applyaction(map, x_size, y_size, next.x, next.y, next.theta, &newx, &newy, &newtheta, mprim, dir, prim);
            // skip action that leads to collision
            if (ret) {
                double disttotarget = (double)sqrt(((newx-goalposeX)*(newx-goalposeX) + (newy-goalposeY)*(newy-goalposeY)));
                if(disttotarget < mindisttotarget){
                    mindisttotarget = disttotarget;
                    
                    *prim_id = prim;
                }
            }
        }
    } else {
        while (pred.prim_before != -1) {
            *prim_id = pred.prim_before;
            pred = prim_from[(int)roundf(pred.x / RES)][(int)roundf(pred.y / RES)];
        }
    }
}

static void planner(
		   double*	map,
		   int x_size,
 		   int y_size,
           float robotposeX,
            float robotposeY,
            float robotposeTheta,
            float goalposeX,
            float goalposeY,
            PrimArray mprim,
            int *prim_id)
{   
    printf("temp=%d\n", temp);
    temp = temp+1;

    *prim_id = 0; /* arbitrary action */
    // printf("x_size %f, y_size %f\n", x_size, y_size);
//     printf("robot: %f %f; ", robotposeX, robotposeY);
//     printf("goal: %d %d;", goalposeX, goalposeY);
    
	/*for now greedily move towards the target, */
	/*but this is where you can put your planner */
	double mindisttotarget = 1000000;
    
    // int prim;
	
    forwardAstar(map, x_size, y_size, robotposeX, robotposeY, robotposeTheta, goalposeX, goalposeY, mprim, prim_id);
    
    // *prim_id = from[(int)(cand.x / RES)][(int)(cand.y / RES)];
    
    // for (prim = 0; prim < NUMOFPRIMS; prim++) {
    //     float newx, newy, newtheta;
    //     bool ret;
    //     ret = applyaction(map, x_size, y_size, robotposeX, robotposeY, robotposeTheta, &newx, &newy, &newtheta, mprim, dir, prim);
    //         /* skip action that leads to collision */
    //     if (ret) {
    //         double disttotarget = (double)sqrt(((newx-goalposeX)*(newx-goalposeX) + (newy-goalposeY)*(newy-goalposeY)));
    //         if(disttotarget < mindisttotarget){
    //             mindisttotarget = disttotarget;
                
    //             *prim_id = prim;
    //         }            
    //     }

    // }
    printf("action %d\n", *prim_id);
    return;
}

/*prhs contains input parameters (3): 
1st is matrix with all the obstacles
2nd is a row vector <x,y> for the robot pose
3rd is a row vector <x,y> for the target pose
plhs should contain output parameters (1): 
1st is a row vector <dx,dy> which corresponds to the action that the robot should make*/

void parseMotionPrimitives(PrimArray mprim)
{
    FILE * fp;
    fp = fopen ("unicycle_8angles.mprim", "r+");
    char skip_c[100];
    int skip_f;
    float resolution;
    int num_angles;
    int num_mprims;
    fscanf(fp, "%s %f", skip_c, &resolution);
    fscanf(fp, "%s %d", skip_c, &num_angles);
    fscanf(fp, "%s %d", skip_c, &num_mprims);

    int i, j, k;
    for (i = 0; i < NUMOFDIRS; ++i) {
        for (j = 0; j < NUMOFPRIMS; ++j) {
            fscanf(fp, "%s %d", skip_c, &skip_f);
            for (k = 0; k < NUMOFINTERSTATES; ++k) {
                fscanf(fp, "%f %f %f", &mprim[i][j][k][0], &mprim[i][j][k][1], &mprim[i][j][k][2]);
            }

        }
    }
}

void mexFunction( int nlhs, mxArray *plhs[], 
		  int nrhs, const mxArray*prhs[] )
     
{

    /* Read motion primtives */
    PrimArray motion_primitives;
    parseMotionPrimitives(motion_primitives);

    /* Check for proper number of arguments */    
    if (nrhs != 3) { 
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Three input arguments required."); 
    } else if (nlhs != 1) {
	    mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required."); 
    } 
        
    /* get the dimensions of the map and the map matrix itself*/     
    int x_size = mxGetM(MAP_IN);
    int y_size = mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    
    /* get the dimensions of the robotpose and the robotpose itself*/     
    int robotpose_M = mxGetM(ROBOT_IN);
    int robotpose_N = mxGetN(ROBOT_IN);
    if(robotpose_M != 1 || robotpose_N != 3){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidrobotpose",
                "robotpose vector should be 1 by 3.");         
    }
    double* robotposeV = mxGetPr(ROBOT_IN);
    float robotposeX = (float)robotposeV[0];
    float robotposeY = (float)robotposeV[1];
    float robotposeTheta = (float)robotposeV[2];
    
    /* get the dimensions of the goalpose and the goalpose itself*/     
    int goalpose_M = mxGetM(GOAL_IN);
    int goalpose_N = mxGetN(GOAL_IN);
    if(goalpose_M != 1 || goalpose_N != 3){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidgoalpose",
                "goalpose vector should be 1 by 3.");         
    }
    double* goalposeV = mxGetPr(GOAL_IN);
    float goalposeX = (float)goalposeV[0];
    float goalposeY = (float)goalposeV[1];
        
    /* Create a matrix for the return action */ 
    ACTION_OUT = mxCreateNumericMatrix( 1, 1, mxINT8_CLASS, mxREAL); 
    int* action_ptr = (int*) mxGetData(ACTION_OUT);

    /* Do the actual planning in a subroutine */
    planner(map, x_size, y_size, robotposeX, robotposeY, robotposeTheta, goalposeX, goalposeY, motion_primitives, &action_ptr[0]);

    return;
    
}





