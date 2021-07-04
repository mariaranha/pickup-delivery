// Project and Analysis of Algorithms
// 
// Problems with connectivity: Minimum Cost k-paths (edge disjoint)
// Given a digraph G with non-negative costs in the arcs,
// two nodes s and t and an integer k, the program obtain
// k edge disjoint paths from s to t with minimum total cost.
// In particular, if k=1 the program obtains a shortest path from s to t.
// The program uses only linear programming and obtain an integer solution
// using the fact that the constraints of the formulation Ax==b (==, <= or >=)
// defines an integer polyhedral (using the fact that A is totally unimodular 
// and b is integer).
//
// Send corrections/comments to Flávio K. Miyazawa

#include <gurobi_c++.h>
#include <iostream>
#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <queue>
#include <lemon/list_graph.h>
#include "mygraphlib.h"
#include <string>
#include "myutils.h"
#include "solver.h"
#include <lemon/concepts/digraph.h>
#include <lemon/preflow.h>
using namespace lemon;
using namespace std;


int cutcount = 0;

// kPaths_Instance put all relevant information in one class.
class kPaths_Instance {
public:
  kPaths_Instance(Digraph &graph,
		  DNodeStringMap &vvname,
		  DNodePosMap &posx,
		  DNodePosMap &posy,
		  ArcValueMap &eweight,
		  DNode &sourcenode,
		  DNode &targetnode,
		  int k);
  Digraph &g;
  DNodeStringMap &vname;
  DNodePosMap &px;
  DNodePosMap &py;
  ArcValueMap &weight;
  int k,nnodes;
  DNode &sourcenode;
  DNode &targetnode;
};

kPaths_Instance::kPaths_Instance(Digraph &graph,
				 DNodeStringMap &vvname,
				 DNodePosMap &posx,
				 DNodePosMap &posy,
				 ArcValueMap &eweight,
				 DNode &nsourcenode,
				 DNode &ntargetnode,
				 int nk):
  g(graph), vname(vvname), px(posx), py(posy), weight(eweight),
  sourcenode(nsourcenode), targetnode(ntargetnode)
{
  nnodes = countNodes(g);
  k = nk;
}

int main(int argc, char *argv[]) 
{
  int k,found;
  Digraph g;  // graph declaration
  string digraph_kpaths_filename, source_node_name, target_node_name;
  DNodeStringMap vname(g);  // name of graph nodes
  DNodePosMap px(g),py(g);  // xy-coodinates for each node
  DNodeColorMap vcolor(g);// color of nodes
  ArcStringMap aname(g);  // name for graph arcs
  ArcColorMap ecolor(g); // color of edges
  ArcValueMap lpvar(g);    // used to obtain the contents of the LP variables
  ArcValueMap weight(g);   // edge weights
  Digraph::ArcMap<GRBVar> x(g); // binary variables for each arc
  vector <DNode> V;
  DNode sourcenode,targetnode;
  int seed=0;
  srand48(1);

  // uncomment one of these lines to change default pdf reader, or insert new one
  set_pdfreader("open");    // pdf reader for Mac OS X
  //set_pdfreader("xpdf");    // pdf reader for Linux
  //set_pdfreader("evince");  // pdf reader for Linux
  //set_pdfreader("open -a Skim.app");
  // double cutoff;   // used to prune non promissing branches (of the B&B tree)
  if (argc!=5) {cout<<endl<<
      "Program to obtain k edge dijoint paths from a node s to a node t," << endl <<
      "with minimum total weight. The particular case when k==1, we have" << endl <<
      "the st-shortest path problem." << endl << endl <<
      "Usage: "<< argv[0]<<"  <digraph_kpaths_filename>  <source_node_name>  <target_node_name>  <k>"<< endl << endl;
    cout << "Example:" << endl <<
      "\t" << argv[0] << " "<< getpath(argv[0])+"../instances/t_100.dig 12 50 1" << endl << endl <<
      "\t" << argv[0] << " "<< getpath(argv[0])+"../instances/t_100.dig 12 50 5" << endl << endl;
    exit(0);}

  digraph_kpaths_filename = argv[1];
  source_node_name = argv[2];
  target_node_name = argv[3];
  k = atoi(argv[4]);

  GRBEnv env = GRBEnv();
  GRBModel model = GRBModel(env);
  model.getEnv().set(GRB_IntParam_Seed, seed);
  model.set(GRB_StringAttr_ModelName, "Oriented k-Paths with GUROBI"); // prob. name
  model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE); // is a minimization problem

  ReadDigraph(digraph_kpaths_filename,g,vname,px,py,weight);

  found=0; // Find the source node in the digraph
  for (DNodeIt v(g);v!=INVALID;++v)
    if(vname[v]==source_node_name){sourcenode=v;found=1;break;}
  if (!found) {cout<<"Could not find source node "<<source_node_name<<endl;exit(0);}
  found=0; // Find the target node in the digraph
  for (DNodeIt v(g);v!=INVALID;++v)
    if(vname[v]==target_node_name){targetnode=v;found=1;break;}
  if (!found) {cout<<"Could not find target node "<<target_node_name<<endl;exit(0);}
    
  kPaths_Instance T(g,vname,px,py,weight,sourcenode,targetnode,k);
  
  // Generate the binary variables and the objective function
  // Add one binary variable for each edge and set its cost in the objective function
  for (ArcIt e(g); e != INVALID; ++e) {
    string varname="x_" + vname[g.source(e)] + "_" + vname[g.target(e)];
    x[e] = model.addVar(0.0, 1.0, weight[e],GRB_CONTINUOUS,varname); }
  model.update(); // run update to use model inserted variables
  try {
    //model.write("model.lp"); system("cat model.lp");

    // Add degree constraint for each node
    for (DNodeIt v(g); v!=INVALID; ++v) {
      GRBLinExpr exprin, exprout;
      for (InArcIt e(g,v); e != INVALID; ++e) exprin += x[e];
      for (OutArcIt e(g,v); e != INVALID; ++e) exprout += x[e];
      if (v==sourcenode)      model.addConstr(exprout - exprin == k );
      else if (v==targetnode) model.addConstr(exprin - exprout == k );
      else                model.addConstr(exprin - exprout == 0 ); }
    model.optimize();
    if (model.get(GRB_IntAttr_SolCount) == 0)  // if could not obtain a solution
      throw "Could not obtain a solution.";
      
    ArcIntMap Path(g); GetSolverValue(g,Path,x); // copy the x to the int vector Paths
    DigraphAttributes GA(g,vname,px,py);
    GA.SetDefaultDNodeAttrib("color=LightGray style=filled width=0.2 height=0.2 fixedsize=true");
    GA.SetColor(sourcenode,"Red");
    GA.SetColor(targetnode,"Cyan");
    GA.SetAttribByValue(0,Path,"color=lightgray style=dashed");
    if (k>=SizeVisualDistinctColor()) // in this case, there is no sufficient nice colors
      GA.SetAttribByValue(1,Path,"color=Blue"); // so, we paint all with blue
    else {DNode v;
      // Get one path from source to target and change the corresponding Path[a's] to i
      for (int i=2;i<=k;i++){
	v = sourcenode;
	while (v!=targetnode) // transform one of the paths with 1's into paths of i's
	  for (OutArcIt a(g,v);a!=INVALID;++a)
	    if(Path[a]==1){ Path[a]=i; v=g.target(a); break;}} 
      for (int i=1;i<=k;i++) 
	GA.SetAttribByValue(i,Path,"color="+ith_VisualDistinctColorName(i-1));
    }
    GA.SetLabel("minimum kPaths cost in graph with "+IntToString(countNodes(g))+
		" nodes and "+IntToString(k)+" paths from node "+
		vname[sourcenode]+" to node "+vname[targetnode]+": "+
		DoubleToString(GetModelValue(model)));
    GA.View();
  } catch (...) {cout << "Error: Could not solve the model." << endl; }
  return 0;
}

