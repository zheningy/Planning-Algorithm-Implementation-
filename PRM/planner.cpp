/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <time.h>
#include <memory>
#include <set>
#include <algorithm>
#include "mex.h"

using namespace std;

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

class Node {
public:
  vector<weak_ptr<Node>> neighbors;
  double cost;
  double *config;
  int id;
  bool visited;
  weak_ptr<Node> before;
  Node(double c, double *conf, int size) : cost(c) {
    config = new double[size];
    memcpy(config, conf, size * sizeof(double));
  }
  ~Node() {
    delete config;
  }
};

vector<shared_ptr<Node>> findNinR(const vector<shared_ptr<Node>> &nodes, const shared_ptr<Node> &center, double r, int dof) {
  vector<shared_ptr<Node>> results;
  for (const auto &node : nodes) {
    if (node == center) continue;
    double d = dist(node->config, center->config, dof);
    if (d < r) {
      results.push_back(node);
    }
  }
  return results;
}

void setId(const vector<shared_ptr<Node>> &nodes, int from, int to) {
  for (const auto &node : nodes) {
    if (node->id == from) node->id = to;
  }
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
    
    //for now just do straight interpolation between start and goal checking for the validity of samples
    //but YOU  WILL WANT TO REPLACE THE CODE BELOW WITH YOUR PLANNER
    
    //RRT
    //vector of 
    //sample random
    double eps = PI / 20;
    int assigner = 0;
    vector<shared_ptr<Node>> nodes;
    auto node_start = make_shared<Node>(0, armstart_anglesV_rad, numofDOFs);
    node_start->id = assigner++;
    auto node_end = make_shared<Node>(100000, armgoal_anglesV_rad, numofDOFs);
    node_end->id = assigner++;
    nodes.push_back(node_start);
    nodes.push_back(node_end);
    for (int i = 0; i < 300000; i++) {
      double *q_rand = new double[numofDOFs];
      for (int j = 0; j < numofDOFs; j++) {
        q_rand[j] = (double)rand()/(double)(RAND_MAX / (PI * 2));
      }
      if (!IsValidArmConfiguration(q_rand, numofDOFs, map, x_size, y_size)) {
        delete[] q_rand;
        continue;
      }
      auto node = make_shared<Node>(100000, q_rand, numofDOFs);
      node->id = assigner++;
      delete[] q_rand;
      auto neighbors = findNinR(nodes, node, 5 * eps, numofDOFs);
      for (const auto &n : neighbors) {
        if (n->id != node->id && collisionFree(n->config, node->config, numofDOFs, eps, map, x_size, y_size)) {
          setId(nodes, n->id, node->id);
          node->neighbors.push_back(weak_ptr<Node>(n));
          n->neighbors.push_back(weak_ptr<Node>(node));
        }
      }
      nodes.push_back(node);
      if (node_start->id == node_end->id) {
        printf("connected! %d, %d \n", node_start->id, node_end->id);
        break;
      }
    }
    
    // A*
    auto cmp = [](const shared_ptr<Node> &n1, const shared_ptr<Node> &n2) { return n1->cost < n2->cost; };
    set<shared_ptr<Node>, decltype(cmp)> open_set(cmp);
    set<shared_ptr<Node>, decltype(cmp)> closed_set(cmp);
    open_set.insert(node_start);
    auto next = *open_set.begin();
    open_set.erase(open_set.begin());
    int j = 0;
    while (next != node_end) {
      closed_set.insert(next);
      for (const auto &n : next->neighbors) {
        auto p = n.lock();
        if (closed_set.find(p) == closed_set.end()) {
          double g_cand = next->cost + dist(p->config, next->config, numofDOFs);
          if (g_cand < p->cost) {
            // printf("cost %f\n", g_cand);
            p->before = next;
            p->cost = g_cand;
            open_set.insert(p);
          }
        }
      }
      next = *open_set.begin();
      open_set.erase(open_set.begin());
    }
    vector<vector<double>> path;
    auto iter = next;
    while (iter) {
      vector<double> p(numofDOFs, 0);
      for (int i = 0; i < numofDOFs; i++) {
        p[i] = iter->config[i];
      }
      path.push_back(p);
      iter = iter->before.lock();
    }
    reverse(path.begin(), path.end());
    *plan = new double*[path.size()];
    *planlength = path.size();

    for (int i = 0; i < *planlength; i++) {
      (*plan)[i] = new double[numofDOFs]; 
      for (int j = 0; j < numofDOFs; j++) {
        (*plan)[i][j] = path[i][j];
      }
    }
 
    
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





