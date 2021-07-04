// Project and Analysis of Algorithms
// Flávio Keidi Miyazawa
// Problems with connectivity: Maximum Bipartite Matching (using digraphs as example,
// arcs have source in one part and target in the other part)
#include <stdio.h>
#include <string>
#include "mygraphlib.h"
#include "myutils.h"
#include "solver.h"
#include <lemon/lp.h>
#include <lemon/list_graph.h>
#include <lemon/concepts/digraph.h>
#include <gurobi_c++.h>
using namespace lemon;
using namespace std;


int main(int argc, char *argv[]) 
{
  Digraph g;  // graph declaration
  DNodeStringMap vname(g);  // name of graph nodes
  DNodePosMap px(g),py(g);  // xy-coodinates for each node
  DNodeColorMap vcolor(g);// color of nodes
  ArcStringMap aname(g);  // name for graph arcs
  ArcColorMap ecolor(g); // color of edges
  ArcValueMap lpvar(g);    // used to obtain the contents of the LP variables
  ArcValueMap weight(g);   // edge weights
  srand48(1);


  // uncomment one of these lines to change default pdf reader, or insert new one
  //set_pdfreader("open");    // pdf reader for Mac OS X
  //set_pdfreader("xpdf");    // pdf reader for Linux
  //set_pdfreader("evince");  // pdf reader for Linux

      
  // double cutoff;   // used to prune non promissing branches (of the B&B tree)
  if (argc!=2) {
    cout<<endl<<"Usage: "<<argv[0]<<" <digraph_matching_filename>"<<endl<<endl<<
    "Example:"<<endl<<
      "\t" << argv[0] << " "<< getpath(argv[0])+"../instances/b_100_10.dig"<<endl<<
      endl;exit(0);}

  string filename = argv[1];
  ReadDigraph(filename,g,vname,px,py,weight);
  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  GRBEnv env = GRBEnv();
  GRBModel model = GRBModel(env);
  model.set(GRB_IntAttr_ModelSense, GRB_MAXIMIZE); // is a maximization problem

  /* ILP variables */
  Digraph::ArcMap<GRBVar> x(g); // variable for connections, 1=connected, 0=not connected
  
  GRBLinExpr expressao;
  for (ArcIt e(g); e != INVALID; ++e) {
    x[e] = model.addVar(0.0, 1.0, weight[e], GRB_CONTINUOUS);
    // Exercise: Using bipartite graphs, explain why we can use continuous
    // variables and still obtain integer solutions
  }
  // After you have inserted the variables run the update command
  model.update();
  
  for (DNodeIt v(g); v!=INVALID; ++v) {
    GRBLinExpr exprin, exprout;
    int n_arcs_in=0,n_arcs_out=0;
    // for each node, the number of arcs leaving is at most 1
    // remember: the graph is bipartite, with arcs going from one part to the other
    for (InArcIt e(g,v); e != INVALID; ++e) {exprin += x[e]; n_arcs_in++;}
    if (n_arcs_in > 0)  {model.addConstr(exprin  <= 1 ); vcolor[v] = ColorCode("Blue");}
    
    // for each node, the number of arcs entering is at most 1 
    for (OutArcIt e(g,v); e != INVALID; ++e) {exprout += x[e]; n_arcs_out++;}
    if (n_arcs_out > 0) {model.addConstr(exprout <= 1 ); vcolor[v] = ColorCode("Red");}
  }
  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  try {
    model.optimize();
    if (model.get(GRB_IntAttr_SolCount) == 0)  // if could not obtain a solution
      throw "Could not obtain a solution.";
    GetSolverValue(g,lpvar,x); // lpvar <-- x
    DigraphAttributes G(g,vname,px,py);
    G.SetDefaultDNodeAttrib("color=Gray");
    G.SetDefaultDNodeAttrib("style=filled");
    G.SetColorByValue(1.0,lpvar,"Blue");
    G.SetColorByValue(0.0,lpvar,"Invis");
    G.SetLabel("Maximum Bipartite Matching = " + DoubleToString(GetModelValue(model)));
    G.View();
  } catch(GRBException e) {
    cerr << "Could not solve the LP model." << endl;
    cerr << "Error code: " << e.getErrorCode() << endl;
    cerr << e.getMessage();
  }
  return 0;
}

