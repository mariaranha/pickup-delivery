//----------------------------------------------------------------------
// Example of an exact program to solve the Minimum Traveling
// Salesman Problem, using LEMON Library and Integer Linear Programming
// with GUROBI Solver.
//
// This program finds a minimum Traveling Salesman Tour (TSP) using an MTZ
// formulation (Miller, Tucker and Zemlin'1960):
//
//       C. E. Miller, A. W. Tucker, and R. A. Zemlin,
//       Integer programming formulations and traveling salesman problems
//       J. ACM, 7 (1960), pp. 326–329.
//
// Obs.: Compare performance with the exponential formulation implemented
//       in ex_tsp.cpp (faster)
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

// This is the type used to obtain the pointer to the problem data. This pointer
// is stored in the branch and cut tree. And when we define separation routines,
// we can recover the pointer and access the problem data again.
class TSP_Data {
public:
  TSP_Data(Graph &graph,
	   NodeStringMap &nodename,
	   NodePosMap &posicaox,
	   NodePosMap &posy,
	   EdgeValueMap &eweight);
  Graph &g;
  int NNodes,NEdges;
  int max_perturb2opt_it; // maximum number of iterations for heuristic Perturb2OPT
  NodeStringMap &vname;
  EdgeStringMap ename;
  NodeColorMap vcolor;
  EdgeColorMap ecolor;
  EdgeValueMap &weight;
  NodePosMap &posx;
  NodePosMap &posy;
  AdjacencyMatrix AdjMat; // adjacency matrix
  vector<Node> BestCircuit; // vector containing the best circuit found
  double BestCircuitValue;
};

TSP_Data::TSP_Data(Graph &graph, NodeStringMap &nodename,
	  NodePosMap &posicaox, NodePosMap &posicaoy, EdgeValueMap &eweight):
  g(graph),vname(nodename),ename(graph),vcolor(graph),ecolor(graph),weight(eweight),
  posx(posicaox),posy(posicaoy),AdjMat(graph,eweight,MY_INF),
  BestCircuit(countEdges(graph)) {
  NNodes=countNodes(this->g); NEdges=countEdges(this->g);
  BestCircuitValue = DBL_MAX; max_perturb2opt_it = 3000; }

int main(int argc, char *argv[]) 
{
  int time_limit;
  char name[1000];
  Graph g;
  EdgeValueMap weight(g);
  NodeStringMap vname(g);
  NodePosMap   posx(g),posy(g);
  string filename;

  int seed=1;


  // uncomment one of these lines to change default pdf reader, or insert new one
  //set_pdfreader("open");    // pdf reader for Mac OS X
  //set_pdfreader("xpdf");    // pdf reader for Linux
  //set_pdfreader("evince");  // pdf reader for Linux

  srand48(seed);
  time_limit = 3600; // solution must be obtained within time_limit seconds
  if (argc!=2) {cout<< endl << "Usage: "<< argv[0]<<" <graph_filename>"<<endl << endl <<
      "Example:" << endl <<
      "\t" << argv[0] << " "<< getpath(argv[0])+"../instances/k_berlin52.gr" << endl <<
      "\t" << argv[0] << " "<< getpath(argv[0])+"../instances/t_70.gr" << endl << endl; exit(0);}
  
  else if (!FileExists(argv[1])) {cout<<"File "<<argv[1]<<" does not exist."<<endl; exit(0);}
  filename = argv[1];
  
  // Read the (undirected) graph
  if (!ReadGraph(filename,g,vname,posx,posy,weight)) 
    {cout<<"Error reading graph file "<<argv[1]<<"."<<endl;exit(0);}
  TSP_Data tsp(g,vname,posx,posy,weight);

  // generate a directed graph with one node duplicated (there is no
  // need to generate such a digraph, but it will be easier to
  // understand the formulation).
  Digraph dg;
  DNode source,destination;
  std::map<Node,DNode> Node_to_DNode;
  ArcValueMap w(dg);
  DNodeStringMap vn(dg);
  DNodePosMap    px(dg),py(dg);
  DNodeColorMap  vcolor(dg); 
  ArcColorMap    acolor(dg); // color of arcs
  
  Node origin;  // only to choose a node to became origin
  for (NodeIt v(g); v!=INVALID; ++v) { origin = v; break;}

  for (NodeIt v(g); v!=INVALID; ++v) {
    Node_to_DNode[v] = dg.addNode(); // mapping from a node of g to dg
    vn[Node_to_DNode[v]] = vname[v];
    px[Node_to_DNode[v]] = posx[v];
    py[Node_to_DNode[v]] = posy[v];
  }
  for (EdgeIt e(g); e!=INVALID; ++e) {
    Node u=g.u(e),v=g.v(e);
    Arc a,b;
    a = dg.addArc(Node_to_DNode[u],Node_to_DNode[v]);
    b = dg.addArc(Node_to_DNode[v],Node_to_DNode[u]);
    w[a] = weight[e];
    w[b] = weight[e];
  }

  //---------------------------------------
  // Build formulation MTZ using digraph
  Digraph::ArcMap<GRBVar> x(dg);
  Digraph::NodeMap<GRBVar> u(dg);
  GRBEnv env = GRBEnv();
  GRBModel model = GRBModel(env);
  model.set(GRB_StringAttr_ModelName, "Symetric TSP using MTZ formulation"); // name to the problem
  model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE); // is a minimization problem
  
  // Add one binary variable for each edge and also sets its cost in the objective function
  for (ArcIt a(dg); a!=INVALID; ++a) {
    sprintf(name,"x_%s_%s",vn[dg.source(a)].c_str(),vn[dg.target(a)].c_str());
    x[a] = model.addVar(0.0, 1.0, w[a],GRB_BINARY,name);  }
  
  for (DNodeIt v(dg);v!=INVALID;++v)
    u[v]=model.addVar(0,tsp.NNodes-1,0,GRB_CONTINUOUS,"");

  model.update(); // run update to use model inserted variables

  // Add degree constraint for each node
  for (DNodeIt v(dg); v!=INVALID; ++v) {
    GRBLinExpr inexpr,outexpr;

    // solution has only one incoming arc for each node
    for (InArcIt a(dg, v); a != INVALID; ++a) inexpr += x[a];
    model.addConstr(inexpr == 1 );

    // solution has only one outgoing arc for each node
    for (OutArcIt a(dg, v); a != INVALID; ++a) outexpr += x[a];
    model.addConstr(outexpr == 1 ); }
  
  for (ArcIt a(dg); a!=INVALID; ++a) {
    if (dg.target(a)!=Node_to_DNode[origin])
      // if arc a is chosen, u[source(a)] + 1 <= u[target(a)]
      model.addConstr(u[dg.source(a)]-u[dg.target(a)]+tsp.NNodes*x[a]<=tsp.NNodes-1); }      

  try {
    if (time_limit >= 0) model.getEnv().set(GRB_DoubleParam_TimeLimit,time_limit);
    model.optimize();
    if (model.get(GRB_IntAttr_SolCount) == 0)  // if could not obtain a solution
      throw "Could not obtain a circuit.";
    for (NodeIt v(g);v!=INVALID;++v) 
      tsp.BestCircuit[(int)ceil(u[Node_to_DNode[v]].get(GRB_DoubleAttr_X)-0.01)] = v;

    DigraphAttributes G(dg,vn,px,py);
    G.SetDigraphAttrib("splines=false");
    G.SetDefaultDNodeAttrib("color=Gray style=filled width=0.2 height=0.2 fixedsize=true");
    SetColorByValue(G,1.0,x,"Blue"); // set color of e based on x[e] values
    SetColorByValue(G,0.0,x,"Invis");
    G.SetDigraphAttrib("label=\"TSP Solution obtained with MTZ formulation with cost "+
		       DoubleToString(GetModelValue(model))+"\"");
    G.View();
  }catch (...) { cout << "Error: Could not obtain a circuit."  << endl; return 1; }
  
}
