//----------------------------------------------------------------------
// Branch and Cut algorithm for the Minimum Cost Steiner Tree
//
// Formulation using non-directed graphs.
//
// It is interesting to compare the performance of the formulation used
// in ex_steiner_directed.cpp (faster) with the one used 
// in ex_steiner_undirected.cpp (slower)
//
// This code is not optimized. For example, for the cutting plane step, it does
// not contract edges with x[e]==1. If this is made, as made for the code in
// ex_tsp.cpp, the performance does not become better than the code in
// ex_steiner_directed.cpp.
//
// Chopra and Rao [CR94] show that the optimal value of the LP relaxation of
// the directed model in ex_steiner_directed.cpp is greater or equal to the
// corresponding value of the undirected formulation in ex_steiner_undirected.cpp
//
// [CR94] Chopra, S. and Rao, M. R. (1994a). The Steiner tree problem
// I: Formulations, compo- sitions and extension of facets.
// Mathematical Programming, 64(2):209-229.
//
// An interesting paper using the directed formulation, using several techniques
// (such as preprocess the input graph to remove edges/nodes, use of better
// inequalities,etc) is given in the following paper:
// 
// [KM98] T. Koch  A. Martin. Solving Steiner tree problems in graphs to optimality.
// Networks, 32: 207-232, 1998.
//
// A more sofisticated code can be seen in the paper
//
// [GKMRS17] G. Gamrath, T. Koch, S.J. Maher, D. Rehfeldt,
// Y. Shinano. SCIP-Jack-a solver for STP and variants with
// parallelization extensions.  Math. Prog. Comp. (2017) 9:231-296.
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


// Steiner_Instance put all relevant information in one class.
class Steiner_Instance {
public:
  Steiner_Instance(Graph &graph,
		   NodeStringMap &vvname,
		   NodePosMap &posx,
		   NodePosMap &posy,
		   EdgeValueMap &eweight,
		   vector <Node> &V); // first nt nodes are terminals
  Graph &g;
  NodeStringMap &vname;
  NodePosMap &px;
  NodePosMap &py;
  EdgeValueMap &weight;
  int nt,nnodes;
  vector <Node> &V; // Node V[0] is the root. Nodes V[1], ... , V[nt-1] are the destination
};

Steiner_Instance::Steiner_Instance(Graph &graph,NodeStringMap &vvname,
	NodePosMap &posx,NodePosMap &posy,EdgeValueMap &eweight,vector <Node> &V):
        g(graph), vname(vvname), px(posx), py(posy), weight(eweight), V(V) {
  nnodes = countNodes(g);
  nt = V.size();
}


class ConnectivityCuts: public GRBCallback
{ Steiner_Instance &T;
  Graph::EdgeMap<GRBVar>& x;
  double (GRBCallback::*solution_value)(GRBVar);
public:
  ConnectivityCuts(Steiner_Instance &T, Graph::EdgeMap<GRBVar>& x) : T(T),x(x)  {    }
protected:
  void callback()
  { // --------------------------------------------------------------------------------
    // get the correct function to obtain the values of the lp variables
    if  (where==GRB_CB_MIPSOL) // if this condition is true, all variables are integer
      {solution_value = &ConnectivityCuts::getSolution;}
    else if ((where==GRB_CB_MIPNODE) &&  
      (getIntInfo(GRB_CB_MIPNODE_STATUS)==GRB_OPTIMAL))// node with optimal fractional solution
      {solution_value = &ConnectivityCuts::getNodeRel;}
    else return; // return, as this code do not take advantage of the other options
    try {
      NodeBoolMap cutmap(T.g);
      EdgeValueMap capacity(T.g);
      for (EdgeIt e(T.g);e!=INVALID;++e) capacity[e]=(this->*solution_value)(x[e]);
      // Although we can consider only cuts between V[0] and other V[i], i!=0,
      // it is interesting to diversify the cuts with the ones that use different
      // variables/edges. So, we change the index of iRoot during the process.
      int iRoot=0; 
      for (int i=1;i<T.nt;i++) {
	// find a mincut between terminal V[iRoot] and other terminal
	double vcut=MinCut(T.g,capacity,T.V[iRoot],T.V[i],cutmap);
	if (vcut>=1-MY_EPS) continue; // else: found violated cut
	GRBLinExpr expr; 
	for (EdgeIt e(T.g);e!=INVALID;++e)
	  if (cutmap[T.g.u(e)] != cutmap[T.g.v(e)])  expr += x[e];
	addLazy( expr >= 1 );
	iRoot=i;} // Change the root terminal to search 'more different' cuts 
    } catch (GRBException e) {
      cout << "Error number: " << e.getErrorCode() << endl;
      cout << e.getMessage() << endl;
    } catch (...) { cout << "Error during callback**" << endl; } }
};

bool ReadSteiner(string filename,Graph &g,NodeStringMap& vname,
		 NodePosMap& posx,NodePosMap& posy,EdgeValueMap& weight,
		 vector <Node> &V){
  string type = GetGraphFileType(filename);
  if (type!="graph"){cout<<"Error: Unknown type of graph: "<<type<<endl;exit(1);}

  GraphTable GT(filename,g); // Read the graph (only nodes and edges)
  NodeIntMap is_terminal(g);
  bool ok = GT.GetColumn("nodename",vname);
  ok = ok && GT.GetColumn("weight",weight);
  ok = ok && GT.GetColumn("terminal",is_terminal);
  ok = ok && GetNodeCoordinates(GT,"posx",posx,"posy",posy);
  int nt=0;
  for (NodeIt v(g);v!=INVALID;++v) if (is_terminal[v]) nt++;
  V.reserve(nt); V.resize(nt);
  int i=0;
  for (NodeIt v(g);v!=INVALID;++v) if (is_terminal[v]) {V[i] = v; i++;}
  return(ok);
}



int main(int argc, char *argv[]) 
{
  int time_limit;
  char name[1000];
  Graph g;
  EdgeValueMap weight(g);
  NodeStringMap vname(g);
  NodePosMap   posx(g),posy(g);
  string filename;
  vector <Node> V;

  int seed=1;

  // uncomment one of these lines to change default pdf reader, or insert new one
  //set_pdfreader("open");    // pdf reader for Mac OS X
  //set_pdfreader("xpdf");    // pdf reader for Linux
  //set_pdfreader("evince");  // pdf reader for Linux

  srand48(seed);
  time_limit = 3600; // solution must be obtained within time_limit seconds
  if (argc!=2) {cout<< endl << "Usage: "<< argv[0]<<" <graph_filename>"<<endl << endl <<
      "Example:" << endl <<
      "\t"<<argv[0]<<" "<<getpath(argv[0])+"../instances/k_att48_13.steiner"<<endl
		    << endl; exit(0);}
  
  else if (!FileExists(argv[1])) {cout<<"File "<<argv[1]<<" does not exist."<<endl; exit(0);}
  filename = argv[1];
  
  ReadSteiner(filename,g,vname,posx,posy,weight,V);
  Steiner_Instance T(g,vname,posx,posy,weight,V);

  Graph::EdgeMap<GRBVar> x(g);
  GRBEnv env = GRBEnv();
  GRBModel model = GRBModel(env);
  model.getEnv().set(GRB_IntParam_LazyConstraints, 1);
  model.getEnv().set(GRB_IntParam_Seed, seed);
  model.set(GRB_StringAttr_ModelName, "Undirected STEINER with GUROBI"); // name to the problem
  model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE); // is a minimization problem
  
  // Add one binary variable for each edge and also sets its cost in the objective function
  for (EdgeIt e(g); e!=INVALID; ++e) {
    sprintf(name,"x_%s_%s",vname[g.u(e)].c_str(),vname[g.v(e)].c_str());
    x[e] = model.addVar(0.0, 1.0, weight[e],GRB_BINARY,name);  }

  model.update(); // run update to use model inserted variables

  // Add degree constraint for each node (sum of solution edges incident to a node is 2)
  for (int i=0;i<T.nt;i++) {
    GRBLinExpr expr;
    for (IncEdgeIt e(g,V[i]); e!=INVALID; ++e) expr += x[e];
    model.addConstr(expr >= 1 );  }

  // model.write("model.lp"); system("cat model.lp");

  try {
    if (time_limit >= 0) model.getEnv().set(GRB_DoubleParam_TimeLimit,time_limit);

    ConnectivityCuts cb = ConnectivityCuts(T , x);
    model.setCallback(&cb);
    model.optimize();
    //note: if heuristic obtained optimal solution, the model is infeasible due to cuttof
    GraphAttributes G(g,vname,posx,posy);
    G.SetDefaultNodeAttrib("color=Gray style=filled");
    for (int i=0;i<T.nt;i++) G.SetColor(T.V[i],"Red");
    SetColorByValue(G,1.0,x,"Blue"); // set color of e based on x[e] values
    SetColorByValue(G,0.0,x,"Invis");
    G.SetLabel("Solution of Steiner Formulation for graph with " 
	       + IntToString(countNodes(g))
	       + " nodes, " + IntToString(T.nt)+" terminals"+
	       ": " + DoubleToString(model.get(GRB_DoubleAttr_ObjVal)));
    G.View();
    
  }catch (...) { cout << "Graph is infeasible"  << endl; return 1; }
  return(1);
}
