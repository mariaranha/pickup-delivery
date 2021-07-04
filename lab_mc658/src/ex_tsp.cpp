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
// There is an very sofisticated branch and cut program (with source code in C) that 
// obtained exact solutions for graphs with 85,900 cities. For more details, see
// the Concorde TSP Solver at http://www.math.uwaterloo.ca/tsp/concorde.html
// and the book
// [1] William J. Cook, In Pursuit of the Traveling Salesman
//     Mathematics at the Limits of Computation, Princeton University Press, 2014.
// [2] Implementing the Dantzig-Fulkerson-Johnson algorithm for large traveling
// salesman problems. Mathematical Programming, 2003, Volume 97, Issue 1-2, pp 91-153.
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
	   weight(eweight),posx(posicaox),posy(posicaoy),
	   AdjMat(graph,eweight,MY_INF),BestCircuit(countEdges(graph)) {
  NNodes=countNodes(this->g);  NEdges=countEdges(this->g);
  BestCircuitValue = DBL_MAX;  max_perturb2opt_it = 100; }


bool Heuristic_2_OPT(AdjacencyMatrix &A,vector<Node> &Circuit,double &BestCircuitValue, int &NNodesCircuit)
{
  double CurrentWeight=0.0,Remove,Insert;
  bool globalimproved,improved;
  vector<Node> CircuitAux(NNodesCircuit);
  int i,j,k,l;
  for (int i=0;i<NNodesCircuit-1;i++) CurrentWeight += A.Cost(Circuit[i],Circuit[i+1]);
  CurrentWeight += A.Cost(Circuit[NNodesCircuit-1],Circuit[0]);

  i=0;   globalimproved = false;
  do { int i1,i2,j1,j2;
    improved=false;
    i1 = i;
    do {
      // one edge is (i1 , i1+1)
      i2 = (i1+1)%NNodesCircuit;
      for (j=2;j<NNodesCircuit-1;j++) {
	j1 = (i1+j)%NNodesCircuit;
	j2 = (j1+1)%NNodesCircuit;
	Remove = A.Cost(Circuit[i1],Circuit[i2])+A.Cost(Circuit[j1],Circuit[j2]);
	Insert = A.Cost(Circuit[i1],Circuit[j1])+A.Cost(Circuit[i2],Circuit[j2]);
	if (Remove-Insert > 0) {
	  k = 0;
	  CircuitAux[k++] = Circuit[i1];
	  for (l=j1;l!=i1;l=(l-1+NNodesCircuit)%NNodesCircuit)
	    CircuitAux[k++] = Circuit[l];
	  for (l=j2;l!=i1;l=(l+1)%NNodesCircuit)
	    CircuitAux[k++] = Circuit[l];

	  for (k=0;k<NNodesCircuit;k++) Circuit[k] = CircuitAux[k];
	  CurrentWeight = CurrentWeight - Remove + Insert;
	  if (CurrentWeight < BestCircuitValue-MY_EPS) {
	    BestCircuitValue = CurrentWeight;
	    globalimproved = true; }
	  improved = true; } }
      i1=(i1+1)%NNodesCircuit;
    } while (i1!=i);
    i = (i+1)%NNodesCircuit;
  }while (improved);
  if (globalimproved) cout << "[Heuristic: 2OPT] New Solution of value " << BestCircuitValue << "\n";
  return(globalimproved);
}

// This routine must be called when the vector x (indexed on the edges) is integer.
// The contained circuit is transformed into a circuit represented by a sequence of nodes.
// The vector x must represent a circuit (must be conected)
bool Update_Circuit(TSP_Data &tsp, Graph::EdgeMap<GRBVar>& x)
{
  NodeNodeMap Adj1(tsp.g), Adj2(tsp.g);
  double CircuitValue;
  Node FirstNode=INVALID,u,ant;
  int n,i,NNodesCircuit;
  
  NNodesCircuit=0; // used to count nodes, only to verify correctness
  // Adj1[v] and Adj2[v] have the two adjacent nodes of v in the circuit
  for (NodeIt v(tsp.g); v != INVALID; ++v) { Adj1[v]=INVALID; Adj2[v]=INVALID; }
  CircuitValue = 0.0;
  for (EdgeIt e(tsp.g); e != INVALID; ++e) { 
    assert(!(NonBinary(x[e].get(GRB_DoubleAttr_X)))); //f cannot be fractional;}
    if (BinaryIsOne(x[e].get(GRB_DoubleAttr_X))) {  // if the edge is in the solution
      Node u,v;
      NNodesCircuit++;
      u = (tsp.g.u(e)); v = (tsp.g.v(e));   // then, obtain the edge nodes u and v
      CircuitValue += tsp.weight[e];
      assert((Adj1[u]==INVALID)||(Adj2[u]==INVALID));
      assert((Adj1[v]==INVALID)||(Adj2[v]==INVALID));  // and say that
      if (Adj1[u]==INVALID) Adj1[u] = v; else Adj2[u] = v; // v is adjacent to u
      if (Adj1[v]==INVALID) Adj1[v] = u; else Adj2[v] = u; } } // and u is adj. to v

  if (CircuitValue > tsp.BestCircuitValue-MY_EPS) return(false);
  // First verify if the circuit is valid
  assert (NNodesCircuit==tsp.NNodes);
  for (NodeIt v(tsp.g);v!=INVALID;++v) assert((Adj1[v]!=INVALID)&&(Adj2[v]!=INVALID));

  // find one node of the circuit
  for (NodeIt v(tsp.g); v != INVALID; ++v) {FirstNode = v; break;}
  
  n = 1;     ant = FirstNode;     u = Adj1[FirstNode];
  // verify if each node have two adjacent nodes
  while (u!=FirstNode) {
    assert(u!=INVALID);
    if (Adj1[u]==ant) {ant = u; u=Adj2[u];}
    else {assert(Adj2[u]==ant); ant=u; u=Adj1[u]; }
    n++;  }
  if (n!=tsp.NNodes) cout << "Solution is not connected" << endl;
  assert(n==tsp.NNodes); 

  // the circuit is ok. So, update the best circuit (remember that at this point
  tsp.BestCircuitValue = CircuitValue; // the new circuit is better than the 
  i = 1;                               // previous best circuit
  tsp.BestCircuit[0] = FirstNode;
  u = Adj2[FirstNode];
  ant = FirstNode;
  while (u!=FirstNode) {
    tsp.BestCircuit[i] = u;      i++;
    if (Adj1[u]==ant) {ant=u; u=Adj2[u];}
    else {ant=u; u=Adj1[u];}    }
  Heuristic_2_OPT(tsp.AdjMat,tsp.BestCircuit,tsp.BestCircuitValue,tsp.NNodes);
  return(true);
}

class subtourelim: public GRBCallback
{ TSP_Data &tsp;
  Graph::EdgeMap<GRBVar>& x;
  double (GRBCallback::*solution_value)(GRBVar);
public:
  subtourelim(TSP_Data &tsp, Graph::EdgeMap<GRBVar>& x) : tsp(tsp),x(x)  {    }
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
    // Stores the edges with fractional values and integer values
    vector<Edge> FracEdges,OneEdges;
    // produces a subgraph h of g, with edges e with x[e]==1 
    // contracted, so we can apply Gomory-Hu tree in a small graph
    for (EdgeIt e(tsp.g); e != INVALID; ++e) {
      if ((this->*solution_value)(x[e]) > 1-MY_EPS)
	OneEdges.push_back(e); // stores the edges with x[e]==1
      else if ((this->*solution_value)(x[e]) > MY_EPS)
	FracEdges.push_back(e); // includes edges with 0 < x[e] < 1
    }// define the subgraph with edges that have x[e]==1

    try {
      // --------------------------------------------------------------------------------
      // Use union-find to contract nodes (to obtain graph where each componente of g is contracted)
      //for (int i=0;i<tsp.NNodes;i++) UFIndexToNode[i]=INVALID;
      Graph::NodeMap<int> aux_map(tsp.g);
      UnionFind<Graph::NodeMap<int> > UFNodes(aux_map);
      for (NodeIt v(tsp.g); v!=INVALID; ++v) UFNodes.insert(v);
      for (vector<Edge>::iterator e_it=OneEdges.begin(); e_it != OneEdges.end(); ++e_it)
	UFNodes.join(tsp.g.u(*e_it),tsp.g.v(*e_it));// No problem if they are in a same component
      // --------------------------------------------------------------------------------
      // Put in a separate set all edges that are not inside a component 
      vector<Edge> CrossingEdges;
      for (EdgeIt e(tsp.g); e != INVALID; ++e) 
	if (UFNodes.find(tsp.g.u(e)) != UFNodes.find(tsp.g.v(e)))
	  CrossingEdges.push_back(e);
      // --------------------------------------------------------------------------------
      // Generate an inverted list UFIndexToNode to find the node that represents a component
      vector<bool> ComponentIndex(tsp.NNodes);
      vector<Node> Index2h(tsp.NNodes);
      for(int i=0;i<tsp.NNodes;i++) ComponentIndex[i]=false;
      for (NodeIt v(tsp.g); v!=INVALID; ++v) ComponentIndex[UFNodes.find(v)]=true;
      // --------------------------------------------------------------------------------
      // Generate graph of components, add one node for each component and edges
      Graph h;
      EdgeValueMap h_capacity(h); 
      for(int i=0;i<tsp.NNodes;i++)  // add nodes to the graph h
	if (ComponentIndex[i]) Index2h[i]=h.addNode(); 
      for (vector<Edge>::iterator e_it=FracEdges.begin(); e_it != FracEdges.end(); ++e_it){
	Node  u = tsp.g.u(*e_it),              v = tsp.g.v(*e_it),
	     hu = Index2h[UFNodes.find(u)],   hv = Index2h[UFNodes.find(v)];
	Edge a = h.addEdge(hu , hv );         // add edges to the graph h
	h_capacity[a] = (this->*solution_value)(x[*e_it]);
	// In the future, change this code to use only one edge, instead of paralel edges
      }
      // --------------------------------------------------------------------------------
      GomoryHu<Graph, EdgeValueMap> ght(h, h_capacity);   ght.run();
      // The Gomory-Hu tree is given as a rooted directed tree. Each node has
      // an arc that points to its father. The root node has father -1.
      // Remember that each arc in this tree represents a cut and the value of
      // the arc is the weight of the corresponding cut. So, if an arc has weight
      // less than 2, then we found a violated cut and in this case, we insert the
      // corresponding constraint.

      NodeBoolMap cutmap(h);
      for (NodeIt u(h); u != INVALID; ++u) {
	GRBLinExpr expr = 0;
	if (ght.predNode(u)==INVALID) continue; // skip the root node
	if (ght.predValue(u) > 2.0 - MY_EPS) continue; // value of the cut is good
	ght.minCutMap(u, ght.predNode(u), cutmap);  // now, we have a violated cut

	// Percorre as arestas que cruzam alguma componente e insere as que pertencem ao corte
	for (vector<Edge>::iterator e_it=CrossingEdges.begin();e_it!=CrossingEdges.end();++e_it){
	  Node u=tsp.g.u(*e_it), v=tsp.g.v(*e_it),
	    hu = Index2h[UFNodes.find(u)], hv=Index2h[UFNodes.find(v)];
	  if (cutmap[hu] != cutmap[hv])  expr += x[*e_it]; 	}
	addLazy( expr >= 2 );  }
    } catch (...) {  cout << "Error during callback..." << endl;   } }
};


void ChangeNode(Node &a,Node &b){ Node c;  c = a; a = b; b = c; }
  
// This routine starts with some solution (current best or a random generated
// solution) and iteratively perturb changing some nodes and applying 2OPT.
bool TSP_Perturb2OPT(TSP_Data &tsp)
{
  Graph *g;
  int nchanges,i,j;
  g = &tsp.g;
  vector<Node> Circuit(tsp.NNodes), BestCircuit(tsp.NNodes);
  double BestCircuitValue;
  //nchanges = 2 (tests with some instances indicate that nchanges=1 is better
  nchanges = 1;

  // Start with a initial solution (if there is no solution, generate any sequence)
  if (tsp.BestCircuitValue < DBL_MAX) {
    BestCircuitValue = tsp.BestCircuitValue;
    for (int i=0; i<tsp.NNodes; i++) BestCircuit[i] = tsp.BestCircuit[i];
  } else {
    int i=0;
    BestCircuitValue = 0.0;
    for (NodeIt v(*g); v!=INVALID; ++v) BestCircuit[i++]=v;
    for (i=0;i<tsp.NNodes;i++) 
      BestCircuitValue += 
	tsp.AdjMat.Cost(BestCircuit[i] , BestCircuit[(i+1)%tsp.NNodes]);
  }

  for (int it=0;it<tsp.max_perturb2opt_it;it++) {
    if (!(it%100)) printf("[Heuristic: Perturbation+2OPT] it = %d (of %d)\n",it+1,tsp.max_perturb2opt_it);
    for (int k=0;k<tsp.NNodes;k++) Circuit[k] = BestCircuit[k];
    for (int nc=0;nc<nchanges;nc++) {
      i = (int) (drand48()*tsp.NNodes);  // get two random nodes and exchange
      j = (int) (drand48()*tsp.NNodes);  // their positions
      if (i!=j) ChangeNode(Circuit[i],Circuit[j]);
    }
    Heuristic_2_OPT(tsp.AdjMat,Circuit,BestCircuitValue,tsp.NNodes);
    if (BestCircuitValue < tsp.BestCircuitValue) { //update the best circuit used
      tsp.BestCircuitValue = BestCircuitValue;     // by the heuristic
      for (int i=0;i<tsp.NNodes;i++) {
	BestCircuit[i] = Circuit[i];
	tsp.BestCircuit[i] = Circuit[i];
      } } }
  return(true);
}


void ViewTspCircuit(TSP_Data &tsp)
{ Graph h;
  NodeStringMap h_vname(h);  // node names
  NodeNodeMap g2h(tsp.g);  // maps a node of g to a node of h
  NodePosMap h_posx(h);
  NodePosMap h_posy(h);

  // Make a graph h (from the nodes in g) inserting only the edges of the circuit
  for (NodeIt v(tsp.g); v!=INVALID; ++v) {
    Node hv=h.addNode();  g2h[v]=hv;
    h_posx[hv]=tsp.posx[v]; h_posy[hv]=tsp.posy[v];  h_vname[hv] = tsp.vname[v]; }
  // insert the edges
  for (int i=0;i<tsp.NNodes;i++) 
    h.addEdge(g2h[tsp.BestCircuit[i]] , g2h[tsp.BestCircuit[(i+1) % tsp.NNodes]]);
  GraphAttributes G(h,h_vname,h_posx,h_posy);
  //G.SetDefaultNodeAttrib("color=gray style=filled"); // display node numbers
  G.SetDefaultNodeAttrib("shape=point"); // do not display node number, only points
  G.SetLabel("TSP tour with cost "+DoubleToString(tsp.BestCircuitValue));
  G.View();
}


int main(int argc, char *argv[]) 
{
  int time_limit;
  char name[1000];
  double cutoff=0.0;
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
      "\t" << argv[0] << " "<< getpath(argv[0])+"../instances/k_att48.gr" << endl << endl; exit(0);}
  
  else if (!FileExists(argv[1])) {cout<<"File "<<argv[1]<<" does not exist."<<endl; exit(0);}
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
    x[e] = model.addVar(0.0, 1.0, weight[e],GRB_BINARY,name);  }

  model.update(); // run update to use model inserted variables

  // Add degree constraint for each node (sum of solution edges incident to a node is 2)
  for (NodeIt v(g); v!=INVALID; ++v) { GRBLinExpr expr;
    for (IncEdgeIt e(g,v); e!=INVALID; ++e) expr += x[e];
    model.addConstr(expr == 2 );  }

  try {
    if (time_limit >= 0) model.getEnv().set(GRB_DoubleParam_TimeLimit,time_limit);

    subtourelim cb = subtourelim(tsp , x);
    model.setCallback(&cb);
    
    tsp.max_perturb2opt_it = 200; //1000; // number of iterations used in heuristic TSP_Perturb2OPT
    TSP_Perturb2OPT(tsp);
    if (tsp.BestCircuitValue < DBL_MAX) cutoff = tsp.BestCircuitValue-MY_EPS; //
    // optimum value for gr_a280=2579, gr_xqf131=566.422, gr_d198=15780, gr_att532=27686
    if (cutoff > 0) model.getEnv().set(GRB_DoubleParam_Cutoff, cutoff );
    model.optimize();
    //note: if heuristic obtained optimal solution, the model is infeasible due to cuttof
    if (model.get(GRB_IntAttr_SolCount) > 0)  // if obtained at least one solution
      Update_Circuit(tsp,x); // Update the circuit in x to tsp circuit variable (if better)
  }catch (...) {
    if (tsp.BestCircuitValue < DBL_MAX) {
      cout << "Heuristic obtained optimum solution"  << endl;}
    else { cout << "Graph is infeasible"  << endl; return 1; }}
  
  if (tsp.BestCircuitValue != DBL_MAX) ViewTspCircuit(tsp); //if heuristic or solver obtained solution
  return(1);
}
