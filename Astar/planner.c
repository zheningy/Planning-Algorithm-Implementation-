/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include "mex.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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

typedef struct 
{
    float x;
    float y;
    float theta;
    float val;
} Point;

typedef struct
{
    float x;
    float y;
    int prim_id;
} Pred;

int floor2log2(int n) {
    if (n <= 1) return 0;
    return floor2log2(n >> 1) + 1;
}

void swap(Point *a, Point *b) {
    Point temp = *a;
    *a = *b;
    *b = temp;
}

int min(int a, int b) {
    if (a < b) return a;
    return b;
}

typedef struct {
    Point *arr;
    int size;
    int capacity;
} Heap;

int heapify(Heap *heap, int idx) {
    int min_idx = -1;
    int left_child = idx * 2 + 1;
    if (left_child >= heap->size) return -1;
    min_idx = left_child;
    int right_child = idx * 2 + 2;
    if (right_child < heap->size && heap->arr[right_child].val < heap->arr[left_child].val)
        min_idx = right_child;
    if (heap->arr[idx].val > heap->arr[min_idx].val) {
        swap(&heap->arr[idx], &heap->arr[min_idx]);
        return min_idx;
    }
    return -1;
}

void heapify_up(Heap *heap, int idx) {
    if (idx < 0 || idx >= heap->size) return;
    heapify(heap, idx);
    if (idx == 0) return;
    heapify_up(heap, (idx-1) / 2);
}

void heapify_down(Heap *heap, int idx) {
    if (idx < 0 || idx >= heap->size) return;
    int min_idx = heapify(heap, idx);
    heapify_down(heap, min_idx);
}

void initHeap(Heap *heap, Point *arr, int size) {
    heap->size = heap->capacity = 0;
    heap->arr = NULL;
    if (size == 0 || !arr) return;
    heap->size = size;
    heap->capacity = size * 2;
    heap->arr = (Point *)malloc(heap->capacity * sizeof(Point));
    memcpy(heap->arr, arr, heap->size * sizeof(Point));
    
    int h = floor2log2(heap->size);
    int i = h, j;
    for (; i >= 0; i--) {
        int start = (1 << i) - 1;
        int end = min(heap->size, (1 << (i+1)) - 1) - 1;
        for (j = start; j <= end; j++) {
            heapify_down(heap, j);
        }
    }
}

Point *find(Heap *heap, float x, float y) {
    if (heap->size == 0) return NULL;
    float epsilon = 1e-3f;
    int i;
    for (i = 0; i < heap->size; i++) {
        Point *p = &heap->arr[i];
        if (fabsf(p->x - x) < epsilon && fabsf(p->y - y) < epsilon) {
            return p;
        }
    }
    return NULL;
}

Point pop_min(Heap *heap) {
    Point p;
    if (heap->size == 0) return p;
    Point result = heap->arr[0];
    heap->size--;
    heap->arr[0] = heap->arr[heap->size];
    heapify_down(heap, 0);
    return result;
}

void push_back(Heap *heap, Point pt) {
    if (heap->size == heap->capacity) {
        heap->capacity *= 2;
        if (heap->capacity == 0) heap->capacity = 1;
        heap->arr = (Point *)realloc(heap->arr, heap->capacity * sizeof(Point));
    }
    heap->arr[heap->size] = pt;
    heapify_up(heap, heap->size++);
}

void printHeap(Heap *heap) {
    int i = 0;
    for (i = 0; i < heap->size; i++) {
        printf("%f %f %f,", heap->arr[i].x, heap->arr[i].y, heap->arr[i].val);
    }
    printf("\n");
}

bool verify(Heap *heap, int idx) {
    if (idx >= heap->size) return true;
    int min_idx = -1;
    int left_child = idx * 2 + 1;
    if (left_child >= heap->size) return true;
    min_idx = left_child;
    int right_child = idx * 2 + 2;
    if (right_child < heap->size && heap->arr[right_child].val < heap->arr[left_child].val)
        min_idx = right_child;
    if (heap->arr[idx].val > heap->arr[min_idx].val) return false;
    bool valid = verify(heap, left_child);
    if (right_child < heap->size) valid &= verify(heap, right_child);
    return valid;
}

void freeHeap(Heap *heap) {
    if (heap->arr) free(heap->arr);
    heap->arr = NULL;
    heap->size = heap->capacity = 0;
}

float **create2DArrayFloat(int x, int y) {
    float **arr = (float **)malloc(x * sizeof(float*));
    arr[0] = (float *)malloc(x * y * sizeof(float));
    int i, j;
    for(i = 1; i < x; i++) {
        arr[i] = arr[0] + i * y;
    }
    printf("x, y: %d %d\n", x, y);
    for (i = 0; i < x; i++) {
        for (j = 0; j < y; j++) {
            arr[i][j] = 1000000.f;
        }
    }
    return arr;
}

int **create2DArrayInt(int x, int y) {
    int **arr = (int **)malloc(x * sizeof(int*));
    arr[0] = (int *)malloc(x * y * sizeof(int));
    int i, j;
    for(i = 1; i < x; i++) {
        arr[i] = arr[0] + i * y;
    }
    printf("x, y: %d %d\n", x, y);
    for (i = 0; i < x; i++) {
        for (j = 0; j < y; j++) {
            arr[i][j] = 0;
        }
    }
    return arr;
}

Pred **create2DArrayPred(int x, int y) {
    Pred **arr = (Pred **)malloc(x * sizeof(Pred*));
    arr[0] = (Pred *)malloc(x * y * sizeof(Pred));
    int i, j;
    for(i = 1; i < x; i++) {
        arr[i] = arr[0] + i * y;
    }
    for (i = 0; i < x; i++) {
        for (j = 0; j < y; j++) {
            Pred pred;
            pred.x = -1.f;
            pred.y = -1.f;
            pred.prim_id = -1;
            arr[i][j] = pred;
        }
    }
    return arr;
}

void free2DArray(void **arr) {
    free(arr[0]);
    free(arr);
}

float heuristic(float robotposeX,
                float robotposeY,
                float robotposeTheta,
                float goalposeX,
                float goalposeY) {
    float x_diff = robotposeX - goalposeX;
    float y_diff = robotposeY - goalposeY;
    return sqrtf(x_diff * x_diff + y_diff * y_diff);
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
    float eps = dist > 25.f ? dist : 1.f;
    printf("using epsilon %f\n", eps);
    static Heap* heap = NULL;
    if (!heap) heap = (Heap *)malloc(sizeof(Heap));
    static float** g = NULL;
    if (!g) g = create2DArrayFloat(x_size, y_size);
    static int** closed_set = NULL;
    if (!closed_set) closed_set = create2DArrayInt(x_size, y_size);
    static Pred** from = NULL;
    if (!from) from = create2DArrayPred(x_size, y_size);
    int i, j;
    for (i = 0; i < x_size; ++i)
    {
        for (j = 0; j < y_size; ++j)
        {
            g[i][j] = 100000.f;
            closed_set[i][j] = 0;
            Pred pred;
            pred.x = -1.f;
            pred.y = -1.f;
            pred.prim_id = -1;
            from[i][j] = pred;
        }
    }
    g[(int)roundf(robotposeX / RES)][(int)roundf(robotposeY / RES)] = 0.f;
    initHeap(heap, NULL, 0);
    double mindisttotarget = 1000000;

    Point p;
    p.x = robotposeX;
    p.y = robotposeY;
    p.theta = robotposeTheta;
    p.val = eps * heuristic(robotposeX, robotposeY, robotposeTheta, goalposeX, goalposeY);
    push_back(heap, p);

    Point cand = pop_min(heap);
    int prim;
    // int cnt = 0;
    while (!(fabsf(cand.x - goalposeX) < 1e-3f && fabsf(cand.y - goalposeY) < 1e-3f)) {
        // if (cnt++ >= 300) break;
        closed_set[(int)roundf(cand.x / RES)][(int)roundf(cand.y / RES)] = 1;
        int dir;
        dir = getPrimitiveDirectionforRobotPose(cand.theta);
        for (prim = 0; prim < NUMOFPRIMS; prim++) {
            float newx, newy, newtheta;
            bool ret;
            ret = applyaction(map, x_size, y_size, cand.x, cand.y, cand.theta, &newx, &newy, &newtheta, mprim, dir, prim);
                /* skip action that leads to collision */
            if (ret && !closed_set[(int)roundf(newx / RES)][(int)roundf(newy / RES)]) {
                float g_cand = g[(int)roundf(cand.x / RES)][(int)roundf(cand.y / RES)] 
                    + sqrtf((newx - cand.x) * (newx - cand.x) + (newy - cand.y) * (newy - cand.y));
                if (g_cand < g[(int)roundf(newx / RES)][(int)roundf(newy / RES)]) {
                    g[(int)roundf(newx / RES)][(int)roundf(newy / RES)] = g_cand;
                    // printf("%f\n", g_cand);
                    Point p;
                    p.x = newx;
                    p.y = newy;
                    p.theta = newtheta;
                    p.val = g_cand + eps * heuristic(newx, newy, newtheta, goalposeX, goalposeY);
                    Pred pred;
                    pred.x = cand.x;
                    pred.y = cand.y;
                    pred.prim_id = prim;
                    from[(int)roundf(newx / RES)][(int)roundf(newy / RES)] = pred;
                    if (!find(heap, p.x, p.y)) push_back(heap, p);
                }    
            }
        }
        // printf("heap size\n", heap->size);
        if (heap->size == 0) break;
        cand = pop_min(heap);
    }

    // find target, back tracing
    Pred pred = from[(int)roundf(cand.x / RES)][(int)roundf(cand.y / RES)];
    if (pred.prim_id == -1) {
        printf("rollback to greedy\n");
        for (prim = 0; prim < NUMOFPRIMS; prim++) {
            float newx, newy, newtheta;
            bool ret;
            int dir;
            dir = getPrimitiveDirectionforRobotPose(cand.theta);
            ret = applyaction(map, x_size, y_size, cand.x, cand.y, cand.theta, &newx, &newy, &newtheta, mprim, dir, prim);
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
        while (!(fabsf(pred.x - robotposeX) < 1e-3f && fabsf(pred.y - robotposeY) < 1e-3f)) {
            pred = from[(int)roundf(pred.x / RES)][(int)roundf(pred.y / RES)];
        }
        *prim_id = pred.prim_id;
    }
    
    freeHeap(heap);
    // free2DArray(closed_set);
    // free2DArray(from);
    // free2DArray(g);
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





