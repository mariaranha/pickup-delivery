// Project and Analysis of Algorithms
// Flávio Keidi Miyazawa
// Problems using only linear programming: Vertex Cover in Bipartite Graphs
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

int vertex_cover_in_bipartite_graph(Graph &g,NodeStringMap &vname,NodeValueMap &cost,
				    NodeIntMap &solution)
{
  int seed=0;
  GRBEnv env = GRBEnv();
  GRBModel model = GRBModel(env);
  model.getEnv().set(GRB_IntParam_Seed, seed);
  model.set(GRB_StringAttr_ModelName, "Vertex Cover in Bipartite Graphs"); // prob. name
  model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE); // is a minimization problem
  Graph::NodeMap<GRBVar> x(g); // binary variables for each node
  try {
    // Add continuous variable x[v] in the interval [0,1] and cost[v] in the objective function
    for (NodeIt v(g); v != INVALID; ++v) 
      x[v] = model.addVar(0.0, 1.0, cost[v],GRB_CONTINUOUS,"X_"+vname[v]);
    model.update();

    for (EdgeIt e(g);e!=INVALID;++e) model.addConstr(x[g.u(e)] + x[g.v(e)] >= 1);
    
    //model.write("model.lp"); system("cat model.lp");
    model.optimize();
    if (model.get(GRB_IntAttr_SolCount) == 0)  // if could not obtain a solution
      throw "Could not obtain a solution.";
    GetSolverValue(g,solution,x);
    return(1);
  } catch (...) {cout << "Error during callback..." << endl; return(0);}
}

// vCover_Instance put all relevant information in one class.
class vCover_Instance {
public:
  vCover_Instance(Graph &graph, NodeStringMap &vvname,
		  NodePosMap &posx, NodePosMap &posy, NodeValueMap &vweight);
  Graph &g;
  NodeStringMap &vname;
  NodePosMap &px, &py;
  NodeValueMap &weight;
  int nnodes; };

// Constructor basically set the number of nodes.
vCover_Instance::vCover_Instance(Graph &graph,NodeStringMap &vvname,
		 NodePosMap &posx,NodePosMap &posy,NodeValueMap &vweight):
  g(graph), vname(vvname), px(posx), py(posy), weight(vweight)
{  nnodes = countNodes(g); }


int main(int argc, char *argv[]) 
{
  Graph g;  // graph declaration
  NodeStringMap vname(g);  // name of graph nodes
  EdgeStringMap ename(g);  // name of graph edges
  NodePosMap px(g),py(g);  // xy-coodinates for each node
  NodeColorMap vcolor(g);// color of nodes
  EdgeColorMap ecolor(g); // color of edges
  NodeIntMap solution(g);  // vetor 0/1 que identifica vertices da solucao
  NodeValueMap weight(g);   // node weights
  vector <Node> V;
  srand48(1);

  // uncomment one of these lines to change default pdf reader, or insert new one
  set_pdfreader("open");    // pdf reader for Mac OS X
  //set_pdfreader("xpdf");    // pdf reader for Linux
  // set_pdfreader("evince");  // pdf reader for Linux
  //set_pdfreader("open -a Skim.app");
  
  if (argc!=2) {cout<<endl<<"Usage: "<< argv[0]<<"  <digraph_vcover_filename>"<< endl << endl;
    cout<<"Example:"<<endl<<
      "\t" << argv[0] << " "<< getpath(argv[0])+"../instances/b_10_20.vertexcover"<<endl<<
      endl;exit(0);}

  ReadGraph(argv[1], g, vname, px, py, weight);
  vCover_Instance T(g,vname,px,py,weight);
  for (EdgeIt e(g); e != INVALID; ++e) ename[e] = vname[g.u(e)]+" , "+vname[g.v(e)];

  cout << "Rede com " << T.nnodes << endl << endl;
  cout << "Computadores e seus costs:" << endl << endl;
  for (NodeIt v(g);v!=INVALID;++v)
    cout << "Computador[ " << vname[v] << " ] com cost " << weight[v] << "." << endl;
  cout << endl << "Conexoes entre computadores:" << endl << endl;
  for (EdgeIt e(g);e!=INVALID;++e)
    cout << "Conexao " << ename[e] << "." << endl;
  cout << endl << endl;

  
  // Generate the binary variables and the objective function
  // Add one binary variable for each edge and set its cost in the objective function

  if (vertex_cover_in_bipartite_graph(g, vname, weight, solution)) {
    // verificacao e apresentacao da solucao obtida

    // Verify if you really have a solution
    for (EdgeIt e(g); e!=INVALID; ++e) 
      if (!(solution[g.u(e)] || solution[g.v(e)])) {
	  cout << "Nao foi obtida uma solucao viavel.\nConexao {"
	       << vname[g.u(e)] << "---" << vname[g.v(e)] << "} nao monitorada."
	       << endl << endl; exit(0);}

    double soma=0.0;
    for (NodeIt v(g);v!=INVALID;++v) soma += solution[v]*weight[v];

    GraphAttributes G(g,vname,px,py);
    G.SetDefaultNodeAttrib("color=Gray style=filled");
    G.SetColorByValue(1,solution,"Blue");
    G.SetColorByValue(0,solution,"Gray");
    G.SetLabel("Solution of Vertex Cover for graph with " 
	       + IntToString(countNodes(g)) + " nodes: " +
	       DoubleToString(soma));
    G.View();

  }else{cout << "Programa linear gerado eh inviavel." << endl;return(1);}
}

