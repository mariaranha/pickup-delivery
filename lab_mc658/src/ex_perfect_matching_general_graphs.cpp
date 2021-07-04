
//----------------------------------------------------------------------
// Example of an exact program to solve the Minimum Perfect Matching in
// General Graphs, using LEMON Library and Linear Programming
// with GUROBI Solver. The formulation does not use integer variables
// and is an example of integer solution obtained by the use of
// cutting planes that (in theory) can be obtained in polynomial time.
// Although separation in polynomial time leads to optimization in
// polynomial time, there is still no proof that using the
// cuts used here leads to a polynomial time algorithm for perfect matching.
//
// Given a graph G=(V,E) with |V| even and edge weights given
// by w:E--> Reals, the formulation is: 
//
// Minimize  Sum_{e in E} w_e * x_e
//   Sum_{e in \delta(v)} x_e == 1     For each node v in V
//   Sum_{e in E[S]} x_e <= (|S|-1)/2  For each S \subseteq V with |S| odd
//   0 <= x_e <= 1    For each edge e in E,
//
// where \delta(S) is the set of edges with exactly one extremity in S
// and E[S] is the set of edges with two extremities in S
// Denote the first set of constraints by "degree constraints" and the
// second set of constraints by "odd set constraints" (or also known
// as "blossoms constraints")
// 
// Obs.: Analogously (see notes), you can replace the constraints
//   Sum_{e in E[S]} x_e <= (|S|-1)/2  For each S \subseteq V with |S| odd
// by
//   Sum_{e in \delta(S)} x_e >= 1     For each S \subseteq V with |S| odd
//
// Theorem [Edmonds'65]. The polytope given by the above formulation is integral.
//
// [Edmonds'65] J. Edmonds. Maximum matching and a polyhedron with 0,1
//              vertices. J. Res. Nat. Bur. Standards Sect. B 69, 125-130
//              
// To solve the separation problem (find a "blossom constraint" for an
// invalid point in polynomial time, the program uses the Gomory-Hu tree
// subroutine, available from LEMON package.
//
// Theorem. Given a point x satisfying the degree constraints, for the
//          perfect matching formulation, but not all blossoms constraints,
//          then, one of the violated blossom constraints is one of the cuts
//          presented in a Gomory-Hu cut tree.
//
// The proof of this theorem is a good exercise. Using this fact,
// the algorithm coded here is straighforward.
//
// For more details, see the slides in the link (in portuguese) for the formulation
// http://www.ic.unicamp.br/~fkm/lectures/proglin.pdf
// and my course notes (also in portuguese) used in the analysis of algorithms
// course.
// 
// It is interesting to mention that in the next reference
//
//   [1] K. Chandrasekaran, L.A. Vegh and S.S. Vempala. The Cutting Plane Method is
//   Polynomial for Perfect Matchings. Mathematics of Operations Research (41), 2015
//
// the authors presented a polynomial time based only on cutting planes and
// linear programming as a black box.
//
// Send comments/corrections to Flavio K. Miyazawa.
//----------------------------------------------------------------------
#include <gurobi_c++.h>
#include <float.h>
#include <math.h>
#include "mygraphlib.h"
#include "myutils.h"
#include "solver.h"
using namespace lemon;

bool insert_blossom_constraints(GRBModel &model,Graph &g,NodeStringMap &vname,
	NodePosMap &px,NodePosMap &py,Graph::EdgeMap<GRBVar>& x)
{
  EdgeValueMap capacity(g);
  for (EdgeIt e(g); e!=INVALID; ++e) capacity[e] = x[e].get(GRB_DoubleAttr_X);
  GomoryHu<Graph, EdgeValueMap > ght(g, capacity);
  ght.run();
  
  // The Gomory-Hu tree is given as a rooted directed tree. Each node has
  // an arc that points to its father. The root node has father -1.
  // Remember that each arc in this tree represents a cut and the value of
  // the arc is the weight of the corresponding cut. So, if an arc has weight
  // less than 1 and each set obtained after removing the arc has odd number
  // of nodes, then we obtain a violated point and we insert the
  // corresponding constraint.
  //
  // To compute the number of nodes for each induced cut (when an edge (u,v) is
  // removed from T), we first make a topological sort of the given tree and
  // we compute, for each node u, the total number of nodes below a node, given
  // by CutSize(u), including the node itself. That is, when the arc u-->predNode(u)
  // is removed, CutSize(u) gives the number of nodes in the component with node u.
  // Given an arc (u-->v), if CutSize[u] is odd, the deletion of arc (u-->v) induces
  // two odd sets.

  //-----------------------------------------------------------------
  // Topological sort of the nodes in T (Gomory-Hu cut tree): arcs have the format: u--->predNode[u]
  NodeIntMap indegree(g);
  NodeIntMap CutSize(g);
  list<Node> ZeroDegreeNodes,topological_sort;
  Node v;
  // Compute the degree of each node in T
  for (NodeIt t(g); t != INVALID; ++t) { indegree[t] = 0; CutSize[t] = 1; } 
  for (NodeIt u(g); u != INVALID; ++u) {
    v = ght.predNode(u);
    if (v==INVALID) continue;// u is the root of T
    indegree[v] = indegree[v]+1; } // increase the indegree of the node

  // Get the nodes with indegree zero
  for (NodeIt u(g); u != INVALID; ++u) if (indegree[u]==0) ZeroDegreeNodes.push_back(u);
  
  while (ZeroDegreeNodes.size()>0) {
    Node u = ZeroDegreeNodes.front();// get u, the first element of the list
    ZeroDegreeNodes.pop_front();     // remove u from the list
    topological_sort.push_back(u);   // insert u in the end of the topological sort
    v = ght.predNode(u);
    if (v!=INVALID) {
      indegree[v]--;                   // decrement the indegree of v
      if (indegree[v]==0) ZeroDegreeNodes.push_back(v);   }  }

  //-----------------------------------------------------------------
  // Compute CutSize[v] for each v
  while (topological_sort.size()>1) { // the last node is the root node 
    Node u = topological_sort.front();// get u, the next node in the topological order
    topological_sort.pop_front();     // remove u 
    v = ght.predNode(u);
    CutSize[v] = CutSize[v] + CutSize[u]; } // update the number of nodes below v (the node v itself is already included
  
  //-----------------------------------------------------------------
  // Insert a blossom cut for each violated odd set. Given arc (u,predNode(u)),
  // if CutSize[u] is odd and the corresponding cut has value < 1, we found a violated cut
  bool inserted_new_cut = false;
  for (NodeIt u(g); u != INVALID; ++u) {
    GRBLinExpr expr;
    if ((CutSize[u]%2==0)||(ght.predValue(u)>1.0-MY_EPS)) continue;  // not a violated cut

    for(GomoryHu<Graph,EdgeValueMap>::MinCutEdgeIt a(ght,u,ght.predNode(u));a!=INVALID;++a) expr += x[a];
    model.addConstr(expr >= 1.0 );
    inserted_new_cut = true;  }

  return inserted_new_cut;
}



// This routine also inserts blossom constraints. It is simpler, but less efficient than the above implementation
bool insert_blossom_constraints_slow(GRBModel &model,
				     Graph &g,
				     NodeStringMap &vname,
				     NodePosMap &px,  // xy-coodinates for each node
				     NodePosMap &py,  // 
				     Graph::EdgeMap<GRBVar>& x)
{
 
  EdgeValueMap capacity(g);
  for (EdgeIt e(g); e!=INVALID; ++e) capacity[e] = x[e].get(GRB_DoubleAttr_X);
  GomoryHu<Graph, EdgeValueMap > ght(g, capacity);
  ght.run();
  bool inserted_new_cut = false;
  for (NodeIt u(g); u != INVALID; ++u) {
    if (ght.predNode(u)==INVALID) continue; // is root of Gomory-Hu tree
    double vcut =  ght.predValue(u);
    if (vcut >= 1.0 - MY_EPS) continue;/* is non violated cut */
    int countNodes = 0; // Count the number of nodes in the side of u
    for(GomoryHu<Graph,EdgeValueMap>::MinCutNodeIt a(ght,u,ght.predNode(u)); a!=INVALID; ++a) countNodes++;
    if (countNodes%2==1){ // number of nodes in the side of is odd
      GRBLinExpr expr;
      for(GomoryHu<Graph,EdgeValueMap>::MinCutEdgeIt e(ght,u,ght.predNode(u), true);e!=INVALID;++e) expr += x[e];
      model.addConstr(expr >= 1 );
      inserted_new_cut = true;    }  }

  return inserted_new_cut;
};



int main(int argc, char *argv[]) {
  Graph g;  // graph declaration
  string    filename;
  NodeStringMap  vname(g);  // name of graph nodes
  EdgeStringMap  ename(g);  // name of graph nodes
  NodePosMap   px(g), py(g); // xy-coodinates for each node
  NodeColorMap vcolor(g);// color of nodes
  EdgeColorMap ecolor(g); // color of edges
  EdgeValueMap vx(g);    // used to obtain the contents of the LP variables
  EdgeValueMap weight(g);   // edge weights
  srand48(1);

  // uncomment one of these lines to change default pdf reader, or insert new one
  //set_pdfreader("open");    // pdf reader for Mac OS X
  //set_pdfreader("xpdf");    // pdf reader for Linux
  //set_pdfreader("evince");  // pdf reader for Linux

  if (argc!=2) {
    cout<< endl
	<< "This program computes a minimum cost perfect matching in a graph\n" 
	<< "(non-necessarily bipartite). The graph must have even number of nodes.\n\n" 
	<< "Usage: "<< argv[0]<<" <graph_filename>"<<endl << endl <<
      "Example:" << endl <<
      "\t" << argv[0] << " "<< getpath(argv[0])+"../instances/k_berlin52.gr" << endl <<
      "\t" << argv[0] << " "<< getpath(argv[0])+"../instances/k_att48.gr" << endl << endl 
	<< endl; exit(0);}
  else if (!FileExists(argv[1])) {cout<<"File "<<argv[1]<<" does not exist."<<endl; exit(0);}
  
  filename = argv[1];
  ReadGraph(filename, g, vname, px, py, weight);
  if (countNodes(g) % 2) {
    cout << "\n\nError: Number of nodes in the graph "<< filename
     << " is odd.\nNumber of nodes must be even." << endl << endl; return 0; }
  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  GRBEnv env = GRBEnv();
  GRBModel model = GRBModel(env);
  model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE); 
  model.getEnv().set(GRB_IntParam_LazyConstraints, 1);
  
  /* ILP variables */
  Graph::EdgeMap<GRBVar> x(g); // variable for connections, 1=connected, 0=not connected

  //Cria as variaveis e seta a funcao de otimizacao para a soma de todos os pesos das arestas escolhidas
  for (EdgeIt e(g);e!=INVALID;++e) x[e]=model.addVar(0.0,1.0,weight[e],GRB_CONTINUOUS);
  model.update();

  //restricao de que a somatoria de todas as arestas ligadas a um vertice v tem que ser igual a 1
  for (NodeIt v(g); v != INVALID; ++v) {  GRBLinExpr expr;
    for (IncEdgeIt e(g, v); e != INVALID; ++e) expr += x[e];
    model.addConstr(expr == 1.0 );
    vcolor[v] = ColorCode("White");  }

  try {
    model.optimize();    // Insert cutting planes until we cannot separate the node x.
    if (model.get(GRB_IntAttr_SolCount) == 0)  // if could not obtain a solution
      throw "Could not obtain a solution.";
    while (insert_blossom_constraints(model,g,vname,px,py,x)) model.optimize();
  } catch(GRBException e){cerr<<"Could not solve the linear program.\n"
    <<"Code: "<<e.getErrorCode()<<" getMessage: "<<e.getMessage()<<endl;exit(0);}

    GraphAttributes G(g,vname,px,py);
    G.SetDefaultNodeAttrib("color=Gray shape=point width=.2 height=.2");
    SetColorByValue(G,1.0,x,"Blue"); // set color of e based on x[e] values
    SetColorByValue(G,0.0,x,"Invis");
    G.SetLabel("Minimum Perfect Matching = " + DoubleToString(GetModelValue(model)));
    G.View();
  return 0;
}

