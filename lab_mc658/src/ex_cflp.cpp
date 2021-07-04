// Project and Analysis of Algorithms
// Flávio Keidi Miyazawa
// Problems with connectivity: Capacitated Facility Location
#include <cstdio>
#include <string>
#include <queue>
#include "mygraphlib.h"
#include "myutils.h"
#include "solver.h"
#include <lemon/lp.h>
#include <lemon/list_graph.h>
#include <lemon/concepts/digraph.h>
#include <gurobi_c++.h>
using namespace lemon;
using namespace std;

bool ReadCFLPInstance(string filename, Digraph &g, DNodeStringMap &vname,
		      DNodePosMap &px, DNodePosMap &py, ArcValueMap  &weight,
		      vector <DNode> &Facility, DNodeValueMap  &capacity, DNodeValueMap  &opencost,
		      vector <DNode> &Client, DNodeValueMap  &demand)
{ // Although capacity and other vectors are only defined for part of the nodes, using this way
  // simplify the codification.
  ifstream file;
  file.open(filename.c_str());
  if (!file) {cout << "File '" << filename << "' does not exist.\n"; exit(0);}

  // Get main parameters (obtain quantity of elements in the next tables)
  StringTable MainParameters(1,file);
  int nf=MainParameters.firstint("nfacilities");
  int nc=MainParameters.firstint("nclients");
  int ne=MainParameters.firstint("nconnectioncosts");
  
  // Read other three tables (info for facilities, clients, and connections)
  StringTable FacilityTable(nf,file);
  StringTable ClientTable(nc,file);
  StringTable ConnectioncostTable(ne,file);
  file.close();

  // Insert Facility (new nodes) in the graph. Facility[i] is the node of row i
  InitGraphTable(FacilityTable,g,Facility); // Insert nodes for each row of FacilityTable
  ReadGraphColumn(FacilityTable,Facility,"facility",vname);
  ReadGraphColumn(FacilityTable,Facility,"capacity",capacity);
  ReadGraphColumn(FacilityTable,Facility,"opencost",opencost);

  // Insert Client (new nodes) in the graph. Client[i] is the node of row i
  InitGraphTable(ClientTable,g,Client); // Insert nodes for each row of ClientTable
  ReadGraphColumn(ClientTable,Client,"client",vname);
  ReadGraphColumn(ClientTable,Client,"demand",demand);

  StringToDNodeMap string2node; for(DNodeIt v(g);v!=INVALID;++v) {
    //cout << "vname: " << vname[v] << endl;
    string2node[vname[v]]=v;
  }

  LineToArcMap arcmap;
  // insert arcs (facility --> client) for each connectioncost
  InitGraphTable(ConnectioncostTable,g,string2node,"facility","client",arcmap); 
  ReadGraphColumn(ConnectioncostTable,arcmap,"connectioncost",weight); // read column weight
  return(true);
}

int main(int argc, char *argv[])
{
  if (argc!=2) {
    cout << endl << "Sintax to read instance from a file:" << endl ;
    cout << "      " << argv[0] << " <filename>" << endl << endl <<
      "Example:" << endl <<
      "\t" << argv[0] << " "<< getpath(argv[0])+"../instances/g_10_50_.5_10.cflp" << endl << endl <<
      "\t" << argv[0] << " "<< getpath(argv[0])+"../instances/g_10_50_.5_100.cflp" << endl << endl;
    exit(0);}

  // I'll duplicate some information read in the above vectors into the graph info,
  // to make things more independent, and perhaps become easier to see
  
  Digraph g;  // graph declaration
  DNodeStringMap vname(g);  // name of graph nodes (facilities and clients)
  ArcStringMap aname(g);  // name of graph nodes (facilities and clients)
  DNodeValueMap demand(g);
  DNodeValueMap capacity(g);
  DNodeValueMap opencost(g);
  DNodeColorMap vcolor(g);// color of nodes
  ArcColorMap   ecolor(g); // color of edges
  ArcValueMap   weight(g);
  DNodePosMap px(g);
  DNodePosMap py(g);
  vector <DNode> Client;
  vector <DNode> Facility;

  // Read the instance and store as a directed graph,
  // with arcs going from the facilities to the clients
  ReadCFLPInstance(string(argv[1]),g,vname,px,py,weight,
		   Facility,capacity,opencost,Client,demand);
  int nf=Facility.size(),nc=Client.size();

  // ------------
  GRBEnv env = GRBEnv();  GRBModel model = GRBModel(env);
  model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE); // is a minimization problem

  /* LPI variables */
  Digraph::ArcMap<GRBVar> x(g); // variable for connections, 1=connected, 0=not connected
  Digraph::NodeMap<GRBVar> y(g); // variable for facilities, 1=open, 0=closed

  // Create variables x[a] and y[fac]
  for (int i=0;i<nf;i++) y[Facility[i]]=model.addVar(0.0,1.0,opencost[Facility[i]],GRB_BINARY);
  for(ArcIt e(g); e != INVALID; ++e) x[e]=model.addVar(0.0,1.0,weight[e],GRB_BINARY);
  model.update();

  // Total demand allocated to a facility is at most its corresponding capacity
  for (int i=0;i<nf;i++) {
    GRBLinExpr expr;
    for (OutArcIt a(g,Facility[i]);a!=INVALID;++a)
      expr+=x[a]*demand[g.target(a)];
    model.addConstr(expr <= capacity[Facility[i]]); }

  // Each client must be allocated to exactly one facility
  for (int j=0;j<nc;j++) {
    GRBLinExpr expr;
    for (InArcIt a(g,Client[j]); a != INVALID; ++a) expr += x[a];
    model.addConstr(expr == 1); }

  // If an arc a = (fac-->cli) is in the solution then, fac must be openned
  for(ArcIt a(g); a!=INVALID; ++a) model.addConstr(x[a] <= y[ g.source(a) ]);

  try {
    model.optimize();
    if (model.get(GRB_IntAttr_SolCount) == 0)  // if could not obtain a solution
      throw "Could not obtain a solution.";
    /* preparing to visualize the solution */
    for (int i=0;i<nf;i++) {
      if (BinaryIsOne(y[Facility[i]].get(GRB_DoubleAttr_X))) vcolor[Facility[i]] = ColorCode("Red");
      else vcolor[Facility[i]] = ColorCode("Gray");
      py[Facility[i]] = 500;     px[Facility[i]] = (double)(1000.0/(nf-1))*i;    }
    
    for (int j=0;j<nc;j++) {
      vcolor[Client[j]] = ColorCode("Blue");
      py[Client[j]] = 0;     px[Client[j]] = (double)(1000.0/(nc-1))*j;   }

    DigraphAttributes G(g,vname,px,py);
    /* coloring arcs */
    SetColorByValue(G,1.0,x,"Red");
    SetColorByValue(G,0.0,x,"Gray");
    G.SetDefaultDNodeAttrib("color=cyan style=filled");
    
    G.View();
  } catch(GRBException e) {
    cerr << "Error: Could not solve the model." << endl;
    cerr << "Error code: " << e.getErrorCode() << endl;
    cerr << e.getMessage(); }
  return 0;
}

