/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "mex.h"

/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ARMSTART_IN	prhs[1]
#define	ARMGOAL_IN     prhs[2]


/* Output Arguments */
#define	PLAN_OUT	plhs[0]
#define	PLANLENGTH_OUT	plhs[1]

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define PI 3.141592654

//the length of each link in the arm (should be the same as the one used in runtest.m)
#define LINKLENGTH_CELLS 10

double factorial(int n) {
  double res = 1;
  int i = n;
  for (; i >= 1; i--) {
    res *= i;
  }
  return res;
}

double dist(double *config1, double *config2, int dof) {
  double distance = 0;
  int j;
  for (j = 0; j < dof; j++){
      if(distance < fabs(config1[j] - config2[j]))
          distance = fabs(config1[j] - config2[j]);
  }
  return distance;
}

double length(double *config, int dof) {
  double distance = 0;
  int j;
  for (j = 0; j < dof; j++){
      distance += config[j] * config[j];
  }
  return sqrt(distance);
}

struct Node_t{
  struct Node_t *parent;
  struct Node_t **children;
  int children_cnt;
  double cost;
  double *config;
};

typedef struct Node_t Node;

void findNN(Node *root, double *target, Node **ans, double *min_dist, int dof) {
  if (!root) return;
  double d = dist(root->config, target, dof);
  if (d < *min_dist) {
    *min_dist = d;
    *ans = root;
  }
  int i = 0;
  for (; i < root->children_cnt; i++) {
    findNN(root->children[i], target, ans, min_dist, dof);
  }
}

void treeSize(Node *root, int *n) {
  (*n)++;
  if (!root->children_cnt) return;
  (*n) += root->children_cnt;
  int i = 0;
  for (; i < root->children_cnt; i++) {
    treeSize(root->children[i], n);
  }
}

void findNinR(Node *root, double *target, Node ***ans, double r, int dof, int *n) {
  if (!root) return;
  double d = dist(root->config, target, dof);
  if (d < r) {
    (*n)++;
    *ans = (Node **)realloc(*ans, (*n) * sizeof(Node *));
    (*ans)[(*n)-1] = root;
  }
  int i = 0;
  for (; i < root->children_cnt; i++) {
    findNinR(root->children[i], target, ans, r, dof, n);
  }
}

void removeChild(Node *parent, Node *child) {
  int i = 0;
  int idx = 0;
  for (; i < parent->children_cnt; i++) {
    if (parent->children[i] == child) {
      idx = i;
      break;
    }
  }
  if (idx < parent->children_cnt - 1) {
    memmove(parent->children + idx, parent->children + idx + 1, (parent->children_cnt - idx - 1) * sizeof(Node *));
  }
  parent->children_cnt--;
}

void freeTree(Node *root) {
  int i = 0;
  for(; i < root->children_cnt; i++) {
    freeTree(root->children[i]);
  }
  free(root);
}

typedef struct {
  int X1, Y1;
  int X2, Y2;
  int Increment;
  int UsingYIndex;
  int DeltaX, DeltaY;
  int DTerm;
  int IncrE, IncrNE;
  int XIndex, YIndex;
  int Flipped;
} bresenham_param_t;


void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY, int x_size, int y_size)
{
    double cellsize = 1.0;
	//take the nearest cell
	*pX = (int)(x/(double)(cellsize));
	if( x < 0) *pX = 0;
	if( *pX >= x_size) *pX = x_size-1;

	*pY = (int)(y/(double)(cellsize));
	if( y < 0) *pY = 0;
	if( *pY >= y_size) *pY = y_size-1;
}


void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params)
{
  params->UsingYIndex = 0;

  if (fabs((double)(p2y-p1y)/(double)(p2x-p1x)) > 1)
    (params->UsingYIndex)++;

  if (params->UsingYIndex)
    {
      params->Y1=p1x;
      params->X1=p1y;
      params->Y2=p2x;
      params->X2=p2y;
    }
  else
    {
      params->X1=p1x;
      params->Y1=p1y;
      params->X2=p2x;
      params->Y2=p2y;
    }

   if ((p2x - p1x) * (p2y - p1y) < 0)
    {
      params->Flipped = 1;
      params->Y1 = -params->Y1;
      params->Y2 = -params->Y2;
    }
  else
    params->Flipped = 0;

  if (params->X2 > params->X1)
    params->Increment = 1;
  else
    params->Increment = -1;

  params->DeltaX=params->X2-params->X1;
  params->DeltaY=params->Y2-params->Y1;

  params->IncrE=2*params->DeltaY*params->Increment;
  params->IncrNE=2*(params->DeltaY-params->DeltaX)*params->Increment;
  params->DTerm=(2*params->DeltaY-params->DeltaX)*params->Increment;

  params->XIndex = params->X1;
  params->YIndex = params->Y1;
}

void get_current_point(bresenham_param_t *params, int *x, int *y)
{
  if (params->UsingYIndex)
    {
      *y = params->XIndex;
      *x = params->YIndex;
      if (params->Flipped)
        *x = -*x;
    }
  else
    {
      *x = params->XIndex;
      *y = params->YIndex;
      if (params->Flipped)
        *y = -*y;
    }
}

int get_next_point(bresenham_param_t *params)
{
  if (params->XIndex == params->X2)
    {
      return 0;
    }
  params->XIndex += params->Increment;
  if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
    params->DTerm += params->IncrE;
  else
    {
      params->DTerm += params->IncrNE;
      params->YIndex += params->Increment;
    }
  return 1;
}



int IsValidLineSegment(double x0, double y0, double x1, double y1, double*	map,
		   int x_size,
 		   int y_size)

{
	bresenham_param_t params;
	int nX, nY; 
    short unsigned int nX0, nY0, nX1, nY1;

    //printf("checking link <%f %f> to <%f %f>\n", x0,y0,x1,y1);
    
	//make sure the line segment is inside the environment
	if(x0 < 0 || x0 >= x_size ||
		x1 < 0 || x1 >= x_size ||
		y0 < 0 || y0 >= y_size ||
		y1 < 0 || y1 >= y_size)
		return 0;

	ContXY2Cell(x0, y0, &nX0, &nY0, x_size, y_size);
	ContXY2Cell(x1, y1, &nX1, &nY1, x_size, y_size);

    //printf("checking link <%d %d> to <%d %d>\n", nX0,nY0,nX1,nY1);

	//iterate through the points on the segment
	get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
	do {
		get_current_point(&params, &nX, &nY);
		if(map[GETMAPINDEX(nX,nY,x_size,y_size)] == 1)
            return 0;
	} while (get_next_point(&params));

	return 1;
}

int IsValidArmConfiguration(double* angles, int numofDOFs, double*	map,
		   int x_size, int y_size)
{
    double x0,y0,x1,y1;
    int i;
    
 	//iterate through all the links starting with the base
	x1 = ((double)x_size)/2.0;
    y1 = 0;
	for(i = 0; i < numofDOFs; i++)
	{
		//compute the corresponding line segment
		x0 = x1;
		y0 = y1;
		x1 = x0 + LINKLENGTH_CELLS*cos(2*PI-angles[i]);
		y1 = y0 - LINKLENGTH_CELLS*sin(2*PI-angles[i]);

		//check the validity of the corresponding line segment
		if(!IsValidLineSegment(x0,y0,x1,y1,map,x_size,y_size))
				return 0;
  }
  return 1;  
}


bool collisionFree(double *config1, double *config2, int dof, double eps, double *map, int x_size, int y_size) {
  double d = dist(config1, config2, dof);
  int steps = d / eps + 1;
  double *deltas = (double *)malloc(dof * sizeof(double));
  double *config = (double *)malloc(dof * sizeof(double));
  int i, j;
  for (i = 0; i < dof; i++) {
    deltas[i] = (config2[i] - config1[i]) / steps;
  }
  bool valid = true;
  for (i = 0; i <= steps; i++) {
    for (j = 0; j < dof; j++) {
      config[j] = config1[j] + deltas[j] * i;
    }
    valid &= IsValidArmConfiguration(config, dof, map, x_size, y_size);
  }
  free(config);
  free(deltas);
  return valid;
}

static void planner(
		   double*	map,
		   int x_size,
 		   int y_size,
           double* armstart_anglesV_rad,
           double* armgoal_anglesV_rad,
	   int numofDOFs,
	   double*** plan,
	   int* planlength)
{
	//no plan by default
	*plan = NULL;
	*planlength = 0;
    
    
    //RRT *
    
    //
    double eps = PI / 15;
    int adapt = 2;
    double gamma = 1;
    int dk = numofDOFs / 2;
    double delta = 2 * factorial(dk) * pow(4 * PI, dk) / factorial(2 * dk + 1);
    if (numofDOFs % 2 == 0) {
      delta = pow(PI, dk) / factorial(dk);
    }
    srand(time(NULL));
    Node *root = (Node *)malloc(sizeof(Node));
    root->parent = NULL;
    root->cost = 0;
    root->children = NULL;
    root->children_cnt = 0;
    root->config = armstart_anglesV_rad;
    int k, i, j;
    for (k = 0; k < 1000000; k++) {
      //if (k % 10000 == 0) printf("searching %d\n", k);
      fflush(stdout);
      // random config
      double *q_rand = (double *)malloc(numofDOFs * sizeof(double));
      bool valid = false;
      while (!valid) {
        for (i = 0; i < numofDOFs; i++) {
          q_rand[i] = (double)rand()/(double)(RAND_MAX / (PI * 2));
        }
        valid = (bool)IsValidArmConfiguration(q_rand, numofDOFs, map, x_size, y_size);
      }

      // extend
      Node *neighbor = NULL;
      double min_dist = 100000;
      findNN(root, q_rand, &neighbor, &min_dist, numofDOFs);
      if (neighbor) {
        double *q_new = (double*)malloc(numofDOFs * sizeof(double));
        memcpy((void *)q_new, (void *)neighbor->config, numofDOFs * sizeof(double));

        valid = false;
        double eps_temp = eps * adapt;
        double *point_to = (double *)malloc(numofDOFs * sizeof(double)); 
        for (i = 0; i < numofDOFs; i++) {
          point_to[i] = q_rand[i] - neighbor->config[i];
        }
        double len = length(point_to, numofDOFs);
        while (!valid) {
          double unit = eps_temp / adapt;
          valid = true;
          for (j = 1; j <= adapt; j++) {
            for (i = 0; i < numofDOFs; i++) {
              q_new[i] = neighbor->config[i] + unit * j * point_to[i] / len;
            }
            if (!IsValidArmConfiguration(q_new, numofDOFs, map, x_size, y_size)) {
              valid = false;
              break;
            }
          }
          eps_temp /= 2;
        }
        free(point_to);

        // check if we found target
        if (dist(q_new, armgoal_anglesV_rad, numofDOFs) < eps) {
          printf("target!\n");
          printf("total samples: %d\n", k);
          Node *iter = neighbor;
          while (iter) {
            (*planlength)++;
            *plan = (double**) realloc(*plan, (*planlength)*sizeof(double*));
            (*plan)[(*planlength)-1] = (double*) malloc(numofDOFs*sizeof(double));
            for (i = 0; i < numofDOFs; i++) {
              (*plan)[(*planlength)-1][i] = iter->config[i];
            }
            iter = iter->parent;
          }
          for (i = 0; i < (*planlength) / 2; i++) {
            double *temp = (*plan)[(*planlength)-1-i];
            (*plan)[(*planlength)-1-i] = (*plan)[i];
            (*plan)[i] = temp;
          }
          break;
        }

        // check if it is a new config
        Node *target = NULL;
        min_dist = 100000;
        findNN(root, q_new, &target, &min_dist, numofDOFs);
        if (min_dist > PI / 50) {
          
          Node *child = (Node *)malloc(sizeof(Node));
          child->cost = neighbor->cost + dist(neighbor->config, q_new, numofDOFs);
          child->children = NULL;
          child->children_cnt = 0;
          child->config = q_new;
          // child->parent = neighbor;

          // neighbor->children_cnt++;
          // neighbor->children = (Node **)realloc(neighbor->children, neighbor->children_cnt * sizeof(Node *));
          // neighbor->children[neighbor->children_cnt-1] = child;

          // RTT-star re-wiring
          Node *min_node2new = neighbor;
          Node **nears = NULL;
          int n = 0;
          int tree_size;
          treeSize(root, &tree_size);
          double r = pow(gamma / delta * log(tree_size) / tree_size, 1 / (double)(numofDOFs));
          if (tree_size < r) r = tree_size;
          findNinR(root, q_new, &nears, r, numofDOFs, &n);
          if (n > 0) {
            for (i = 0; i < n; i++) {
              Node *cand = nears[i];
              if (!collisionFree(cand->config, q_new, numofDOFs, eps, map, x_size, y_size)) continue;
              double cand_cost = cand->cost + dist(cand->config, q_new, numofDOFs);
              if (cand_cost < child->cost) {
                child->cost = cand_cost;
                min_node2new = cand;
              }
            }
          }
          child->parent = min_node2new;
          min_node2new->children_cnt++;
          min_node2new->children = (Node **)realloc(min_node2new->children, min_node2new->children_cnt * sizeof(Node *));
          min_node2new->children[min_node2new->children_cnt-1] = child;

          if (n > 0) {
            for (i = 0; i < n; i++) {
              Node *cand = nears[i];
              if (!collisionFree(cand->config, q_new, numofDOFs, eps, map, x_size, y_size)) continue;
              if (cand == min_node2new) continue;
              double cand_cost = child->cost + dist(q_new, cand->config, numofDOFs);
              if (cand_cost < cand->cost) {
                cand->cost = cand_cost;
                if (cand->parent) removeChild(cand->parent, cand);
                cand->parent = child;
                child->children_cnt++;
                child->children = (Node **)realloc(child->children, child->children_cnt * sizeof(Node *));
                child->children[child->children_cnt-1] = cand;
              }
            }
          }
          if (nears) free(nears);
        }
      }
     
    }
    freeTree(root); 
    
    return;
}

//prhs contains input parameters (3): 
//1st is matrix with all the obstacles
//2nd is a row vector of start angles for the arm 
//3nd is a row vector of goal angles for the arm 
//plhs should contain output parameters (2): 
//1st is a 2D matrix plan when each plan[i][j] is the value of jth angle at the ith step of the plan
//(there are D DoF of the arm (that is, D angles). So, j can take values from 0 to D-1
//2nd is planlength (int)
void mexFunction( int nlhs, mxArray *plhs[], 
		  int nrhs, const mxArray*prhs[])
     
{ 
    
    /* Check for proper number of arguments */    
    if (nrhs != 3) { 
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Three input arguments required."); 
    } else if (nlhs != 2) {
	    mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required."); 
    } 
        
    /* get the dimensions of the map and the map matrix itself*/     
    int x_size = (int) mxGetM(MAP_IN);
    int y_size = (int) mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    
    /* get the start and goal angles*/     
    int numofDOFs = (int) (MAX(mxGetM(ARMSTART_IN), mxGetN(ARMSTART_IN)));
    if(numofDOFs <= 1){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidnumofdofs",
                "it should be at least 2");         
    }
    double* armstart_anglesV_rad = mxGetPr(ARMSTART_IN);
    if (numofDOFs != MAX(mxGetM(ARMGOAL_IN), mxGetN(ARMGOAL_IN))){
        	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidnumofdofs",
                "numofDOFs in startangles is different from goalangles");         
    }
    double* armgoal_anglesV_rad = mxGetPr(ARMGOAL_IN);
        
    //call the planner
    double** plan = NULL;
    int planlength = 0;
    
    planner(map,x_size,y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength); 
    
    printf("planner returned plan of length=%d\n", planlength); 
    
    /* Create return values */
    if(planlength > 0)
    {
        PLAN_OUT = mxCreateNumericMatrix( (mwSize)planlength, (mwSize)numofDOFs, mxDOUBLE_CLASS, mxREAL); 
        double* plan_out = mxGetPr(PLAN_OUT);        
        //copy the values
        int i,j;
        for(i = 0; i < planlength; i++)
        {
            for (j = 0; j < numofDOFs; j++)
            {
                plan_out[j*planlength + i] = plan[i][j];
            }
        }
    }
    else
    {
        PLAN_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)numofDOFs, mxDOUBLE_CLASS, mxREAL); 
        double* plan_out = mxGetPr(PLAN_OUT);
        //copy the values
        int j;
        for(j = 0; j < numofDOFs; j++)
        {
                plan_out[j] = armstart_anglesV_rad[j];
        }     
    }
    PLANLENGTH_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)1, mxINT8_CLASS, mxREAL); 
    int* planlength_out = (int *)mxGetPr(PLANLENGTH_OUT);
    *planlength_out = planlength;
    
    return;
}





