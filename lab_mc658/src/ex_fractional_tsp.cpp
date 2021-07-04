//----------------------------------------------------------------------
// Example of an exact program to solve the Minimum Traveling
// Salesman Problem, using LEMON Library and Integer Linear Programming
// with GUROBI Solver.
//
// This program finds a minimum Traveling Salesman Tour (TSP) via a branch
// and cut approach. The formulation used is given in the page 100 of the
// slides in the link (in portuguese)
//
// http://www.ic.unicamp.br/~fkm/lectures/proglin.pdf
//
// To obtain the cuts that are violated, it uses the Gomory-Hu subroutine, 
// available from LEMON package. For a short explanation why it is interesting
// to use Gomory-Hu tree, see pages 141-142 of the above slides (in portuguese).
//
// OBS.: The edge costs in the graphs available in the same directory do not have the
// same costs computed by TSPLIB
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

TSP_Data::TSP_Data(Graph &graph,NodeStringMap &nodename,
	NodePosMap &posicaox,NodePosMap &posicaoy,EdgeValueMap &eweight):
      g(graph),vname(nodename),ename(graph),vcolor(graph),ecolor(graph),
      weight(eweight),posx(posicaox),posy(posicaoy),AdjMat(graph,eweight,MY_INF),
      BestCircuit(countEdges(graph)) {
  NNodes=countNodes(this->g);  NEdges=countEdges(this->g);
  BestCircuitValue = DBL_MAX; max_perturb2opt_it = 3000; }


int main(int argc, char *argv[]) 
{
  int time_limit;
  char name[1000];
  Graph g;
  EdgeValueMap weight(g),fracX(g);
  NodeStringMap vname(g);
  NodePosMap   posx(g),posy(g);
  string filename;
  bool foundviolatedcut;
  int seed=1; srand48(seed);
  // uncomment one of these lines to change default pdf reader, or insert new one
  //set_pdfreader("open");    // pdf reader for Mac OS X
  //set_pdfreader("xpdf");    // pdf reader for Linux
  //set_pdfreader("evince");  // pdf reader for Linux
  time_limit = 3600; // solution must be obtained within time_limit seconds
  if (argc!=2) {cout<< endl << "Usage: "<< argv[0]<<" <graph_filename>"<<endl << endl <<
      "Example:" << endl <<
      "\t" << argv[0] << " "<< getpath(argv[0])+"../instances/k_berlin52.gr" << endl <<
      "\t" << argv[0] << " "<< getpath(argv[0])+"../instances/k_att48.gr" << endl <<
      endl; exit(0);}
  
  else if (!FileExists(argv[1]))
    {cout<<"File "<<argv[1]<<" does not exist."<<endl;exit(0);}
  filename = argv[1];
  
  // Read the graph
  if (!ReadGraph(filename,g,vname,posx,posy,weight)) 
    {cout<<"Error reading graph file "<<argv[1]<<"."<<endl;exit(0);}

  TSP_Data tsp(g,vname,posx,posy,weight); 
  Graph::EdgeMap<GRBVar> x(g);
  GRBEnv env = GRBEnv();
  GRBModel model = GRBModel(env);
  model.getEnv().set(GRB_IntParam_LazyConstraints, 1);
  model.getEnv().set(GRB_IntParam_Seed, seed);
  model.set(GRB_StringAttr_ModelName, "Undirected TSP with GUROBI"); // name to the problem
  model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE); // is a minimization problem
  
  // Add one binary variable for each edge and also sets its cost in the objective function
  for (EdgeIt e(g); e!=INVALID; ++e) {
    sprintf(name,"x_%s_%s",vname[g.u(e)].c_str(),vname[g.v(e)].c_str());
    x[e] = model.addVar(0.0, 1.0, weight[e],GRB_CONTINUOUS,name);  }
  model.update(); // run update to use model inserted variables

  // Add degree constraint for each node (sum of solution edges incident to a node is 2)
  for (NodeIt v(g); v!=INVALID; ++v) {
    GRBLinExpr expr;
    for (IncEdgeIt e(g,v); e!=INVALID; ++e) expr += x[e];
    model.addConstr(expr == 2 );  }

  do {
    foundviolatedcut = false;
    try {
      model.optimize();
      if (model.get(GRB_IntAttr_SolCount) == 0)  // if could not obtain a solution
	throw "Could not obtain a circuit.";
    }catch (...)
      { cout << "Error: Could not obtain a fractional solution."  << endl; return 1; }
    for (EdgeIt e(g); e!=INVALID; ++e) fracX[e] = x[e].get(GRB_DoubleAttr_X);

    
    GomoryHu<Graph, EdgeValueMap> ght(g, fracX);   ght.run();
    // The Gomory-Hu tree is given as a rooted directed tree. Each node has
    // an arc that points to its father. The root node has father -1.
    // Remember that each arc in this tree represents a cut and the value of
    // the arc is the weight of the corresponding cut. So, if an arc has weight
    // less than 2, then we found a violated cut and in this case, we insert the
    // corresponding constraint.
    // This code can be improved, by replacing sequence of edges with x's value 1
    // by only one edge (as made in the code of ex_tsp_gurobi.cpp).

    // Add one constraint for each violated cut in the GH cut tree
    NodeBoolMap cutmap(g);
    for (NodeIt u(g); u != INVALID; ++u) {
      GRBLinExpr expr = 0;
      if (ght.predNode(u)==INVALID) continue; // skip the root node
      if (ght.predValue(u) > 2.0 - MY_EPS) continue; // value of the cut is good
      foundviolatedcut = true;
      ght.minCutMap(u, ght.predNode(u), cutmap);  // now, we have a violated cut
      for (EdgeIt e(g); e!=INVALID; ++e) 
	if (cutmap[g.u(e)] != cutmap[g.v(e)]) // extremities of e are in different parts
	  expr += x[e]; 
      model.addConstr(expr >= 2 ); }
  } while (foundviolatedcut);

    GraphAttributes G(g,vname,posx,posy);
    //G.SetDefaultNodeAttrib("color=gray style=filled");
    G.SetDefaultNodeAttrib("shape=point"); // do not display node number, only points
    SetLabelNonZero(G,x);
    SetColorByValue(G,1.0,x,"Blue"); // All integer vars have color Blue
    SetColorByInterval(G,0+MY_EPS,1-MY_EPS,x,"Red"); // Fractional vars have color Red
    SetColorByValue(G,0.0,x,"Invis"); // Other colors are not shown
    G.SetLabel("Solution of TSP Formulation relaxing integer constraint. "
	       "Graph with "+ IntToString(countNodes(g))+ " nodes:" +
	       DoubleToString(model.get(GRB_DoubleAttr_ObjVal)));
    G.View();
}
