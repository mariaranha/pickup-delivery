//----------------------------------------------------------------------
// Example of an exact program to solve the Vehicle Routing
// Problem, using LEMON Library and Integer Linear Programming
// with GUROBI Solver.
//
// This program finds a solution for the Minimum Capacitated
// Vehicle Routing Problem (VRP) via a branch and cut approach.
// The formulation is based on the chapter
// Branch and Cut Algorithms for the Capacitated VRP, D. Naddef and G. Rinaldi,
// of the book "The Vehicle Routing Problem" book (Toth and Vigo eds.),
// formulation (3.1)--(3.7), but using a simple approach to obtain some
// restricted rounded capacity inequalities. For more details, see the above chapter.
// To obtain an improved algorithm, there is a C++ code, available from J. Lysgaard,
// with several separation routines for the Capacitated Vehicle Routing Problem.
// For the code and paper, see the following references:
//
//    [1] Lysgaard, J.: CVRPSEP: A package of separation routines for the
//    capacitated vehicle routing problem, 2003. Available at www.asb.dk/~lys 
//    [2] Lysgaard,J.,Letchford,A.,Eglese,R.: A new branch-and-cut algorithm for
//    the capacitated vehicle routing problem. Math. Prog. 100 (2), 423-445 (2004)
//
// Send comments/corrections to Flavio K. Miyazawa.
//----------------------------------------------------------------------
#include <assert.h>
#include <gurobi_c++.h>
#include <float.h>
#include <math.h>
#include <set>
#include <lemon/list_graph.h>
#include <lemon/unionfind.h>
#include <lemon/gomory_hu.h>
#include <lemon/adaptors.h>
#include <lemon/connectivity.h>
#include "mygraphlib.h"
#include "myutils.h"
#include "solver.h"
using namespace std;

// This is the type used to obtain the pointer to the problem data. This pointer
// is stored in the branch and cut tree. And when we define separation routines,
// we can recover the pointer and access the problem data again.
class VRP_Data {
public:
  VRP_Data(Graph &graph,
	   NodeStringMap &nodename,
	   NodePosMap &posicaox,
	   NodePosMap &posy,
	   EdgeValueMap &eweight,
	   string depot_node_name,
	   int number_of_routes,
	   double capacity,
	   NodeValueMap &vdemand
	   );
  Graph &g;
  int NNodes,NEdges;
  Node depot;
  int NRoutes;
  NodeStringMap &vname;
  EdgeStringMap ename;
  NodeColorMap vcolor;
  EdgeColorMap ecolor;
  EdgeValueMap &weight;
  double capacity;
  NodeValueMap &demand;
  NodePosMap &posx;
  NodePosMap &posy;
};

VRP_Data::VRP_Data(Graph &graph,NodeStringMap &nodename,NodePosMap &posicaox,
		   NodePosMap &posicaoy,EdgeValueMap &eweight,string depot_node_name,
		   int number_of_routes, double vcapacity, NodeValueMap &vdemand):
    g(graph),vname(nodename),ename(graph),vcolor(graph),ecolor(graph),weight(eweight),
    capacity(vcapacity),demand(vdemand),posx(posicaox),posy(posicaoy){
  this->NRoutes = number_of_routes;
  NNodes=countNodes(this->g);   NEdges=countEdges(this->g);
  bool founddepot=false;
  for (NodeIt v(this->g); v!=INVALID; ++v)
    if (vname[v] == depot_node_name) {this->depot = v; founddepot=true; break;}
  if (!founddepot)
    {cout<<"Could not find the depot node "<<depot_node_name<<"."<<endl;exit(0); }
}

class subtourelim: public GRBCallback
{ VRP_Data &cvrp;
  Graph::EdgeMap<GRBVar>& x;
  double (GRBCallback::*solution_value)(GRBVar);
public:
  subtourelim(VRP_Data &cvrp, Graph::EdgeMap<GRBVar>& x) : cvrp(cvrp),x(x)  {    }
protected:
  void callback()
  { // --------------------------------------------------------------------------------
    // get the correct function to obtain the values of the lp variables
    if  (where==GRB_CB_MIPSOL) // if this condition is true, all variables are integer
      {solution_value = &subtourelim::getSolution;}
    else if ((where==GRB_CB_MIPNODE) &&  
      (getIntInfo(GRB_CB_MIPNODE_STATUS)==GRB_OPTIMAL))// node with optimal fractional solution
      {solution_value = &subtourelim::getNodeRel;}
    else return; // return, as this code do not take advantage of the other options
    // --------------------------------------------------------------------------------
    try {
      // --------------------------------------------------------------------------------
      // Idea: use the support graph G*, obtained from the edges from G where x[e]>0
      // and without depot node.
      // Each component C of G* must satisfy
      //          x(\delta(C)) >= ceil(demand(C)/capacity)
      // where \delta(C) is the set of edges of G with exactly one extremity in C.
      // We use union-find to obtain the components of G*
      Graph::NodeMap<int> aux_map(cvrp.g);
      UnionFind<Graph::NodeMap<int> > UFNodes(aux_map);
      double x_e_cut_threshold=1.0/2.0-MY_EPS; // tried x_e_cut_threshold with values
      // (1-MY_EPS), (1.0/2.0-MY_EPS), (1.0/3.0-MY_EPS) and MY_EPS  for some graphs
      for (NodeIt v(cvrp.g); v!=INVALID; ++v) UFNodes.insert(v);
      for (EdgeIt e(cvrp.g); e != INVALID; ++e) {
	if ((cvrp.g.u(e)!=cvrp.depot) && (cvrp.g.v(e)!=cvrp.depot) &&
	    ((this->*solution_value)(x[e])> x_e_cut_threshold))
	  UFNodes.join(cvrp.g.u(e),cvrp.g.v(e)); }
      
      //compute total demand of each component
      vector <double> totdemand(cvrp.NNodes);
      for (int i=0; i<cvrp.NNodes;i++) totdemand[i] = 0.0;
      for (NodeIt v(cvrp.g); v!=INVALID; ++v) {
	int comp = UFNodes.find(v);
	if ((comp<0)||(comp>=cvrp.NNodes))
	  {cout<<"Error in index of UnionFind in file ex_vrp.cpp.\n";exit(0);}
	totdemand[comp] += cvrp.demand[v]; }

      for (NodeIt v(cvrp.g); v!=INVALID; ++v){
	Node u;	GRBLinExpr expr;
	double Demand_Comp_v, Cut_Comp_v=0.0;
	int Comp_v = UFNodes.find(v),Comp_Depot = UFNodes.find(cvrp.depot);

	if (Comp_v==Comp_Depot)continue;//Processed comps are joined to the Depot.
	
	for (EdgeIt e(cvrp.g); e != INVALID; ++e) { // e={a,b}
	  int Comp_a = UFNodes.find(cvrp.g.u(e)), Comp_b = UFNodes.find(cvrp.g.v(e));
	  if (Comp_a == Comp_b) continue;
	  if ((Comp_a==Comp_v) || (Comp_b==Comp_v)) {
	    expr += x[e];
	    Cut_Comp_v += (this->*solution_value)(x[e]); }}
	Demand_Comp_v = totdemand[UFNodes.find(v)];
	if (Cut_Comp_v < 2*ceil(Demand_Comp_v / cvrp.capacity) - MY_EPS) 
	  addLazy( expr >= 2*ceil(Demand_Comp_v / cvrp.capacity) );
	UFNodes.join(cvrp.depot,v); // Make v processed
      }
    } catch (...) { cout << "Error during callback..." << endl; } }
};

int main(int argc, char *argv[]) 
{
  int time_limit,number_of_routes;
  Graph g;
  EdgeValueMap weight(g),X(g);
  NodeValueMap demand(g);
  NodeStringMap vname(g);
  NodePosMap   posx(g),posy(g);
  string filename,depot_node_name;
  double capacity;
  int seed=1;  srand48(seed);
  // uncomment one of these lines to change default pdf reader, or insert new one
  //set_pdfreader("open");    // pdf reader for Mac OS X
  //set_pdfreader("xpdf");    // pdf reader for Linux
  //set_pdfreader("evince");  // pdf reader for Linux

  time_limit = 3600; // solution must be obtained within time_limit seconds

  if (argc!=5) {cout<<endl<<
      "Usage: "<< argv[0] << "  "
      "<graph_filename>  <Depot>  <number_of_routes>  <vehicle capacity>\n\n" <<
      "Example:" << endl <<
      "\t" << argv[0] << " "<< getpath(argv[0])+"../instances/k_att48.gr 40 4 20" << endl << endl; exit(0);}

  filename = argv[1];
  depot_node_name = argv[2];
  number_of_routes = atoi(argv[3]);
  capacity = atoi(argv[4]);
  if (!FileExists(filename)) {cout<<"File "<<argv[1]<<" does not exist."<<endl; exit(0);}
  
  // Read the graph
  if (!ReadGraph(filename,g,vname,posx,posy,weight)) 
    {cout<<"Error reading graph file "<<argv[1]<<"."<<endl;exit(0);}

  // All nodes have demand 1. In the future, adapt routine to read
  // a file that already contains the demand for each node.
  for (NodeIt v(g); v!=INVALID; ++v) demand[v] = 1.0; 

  VRP_Data cvrp(g,vname,posx,posy,weight,depot_node_name,number_of_routes,capacity,demand);
  
  Graph::EdgeMap<GRBVar> x(g);
  GRBEnv env = GRBEnv();
  GRBModel model = GRBModel(env);
  model.getEnv().set(GRB_IntParam_LazyConstraints, 1);
  model.getEnv().set(GRB_IntParam_Seed, seed);
  model.set(GRB_StringAttr_ModelName, "Undirected Capacitated VRP"); // problem name
  model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE); // is a minimization problem
  
  // Add one binary var. for each edge and also sets its cost in the objective function
  for (EdgeIt e(g); e!=INVALID; ++e) {
    string varname="x_" + vname[g.u(e)] + "_" + vname[g.v(e)];
    if ((g.u(e)==cvrp.depot) || (g.v(e)==cvrp.depot))
      x[e] = model.addVar(0.0, 2.0, weight[e],GRB_INTEGER,varname); 
    else
      x[e] = model.addVar(0.0, 1.0, weight[e],GRB_BINARY,varname);}
  model.update(); // must run update after the insertion of variables

  // Add degree constraint for each node (is 2, except for depot, that is 2*(#of_routes)
  for (NodeIt v(g); v!=INVALID; ++v) { GRBLinExpr expr;
    if (v!=cvrp.depot) {
      for (IncEdgeIt e(g,v); e!=INVALID; ++e) expr += x[e];
      model.addConstr(expr == 2 );}
    else {
      for (IncEdgeIt e(g,v); e!=INVALID; ++e) expr += x[e];
      model.addConstr(expr == 2*cvrp.NRoutes );}}

  try {
    if (time_limit >= 0) model.getEnv().set(GRB_DoubleParam_TimeLimit,time_limit);

    subtourelim cb = subtourelim(cvrp , x);
    model.setCallback(&cb);
    model.optimize();
    if (model.get(GRB_IntAttr_SolCount) == 0)  // if could not obtain a solution
      throw "Could not obtain a solution.";
    cout << "Best Solution cost: "<< GetModelValue(model) << endl;

    GraphAttributes G(g,vname,posx,posy);
    G.SetDefaultNodeAttrib("color=Gray style=filled");
    G.SetColor(cvrp.depot,"Green");
    SetColorByValue(G,1.0,x,"Blue"); // set color of e based on x[e] values
    SetColorByValue(G,2.0,x,"Red");
    SetColorByValue(G,0.0,x,"Invis");
    G.SetLabel("Solution of VRP Formulation for graph with " 
	       + IntToString(countNodes(g))
	       + " nodes, " + IntToString(cvrp.NRoutes)+" routes, "
	       "Depot=" + vname[cvrp.depot] + ", " +
	       "Capacity=" + DoubleToString(cvrp.capacity) +
	       ": " + DoubleToString(model.get(GRB_DoubleAttr_ObjVal)));
    G.View();

  } catch (...) {  cout << "Graph is infeasible"  << endl; return 1; }
}
