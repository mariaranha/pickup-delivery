
//----------------------------------------------------------------------
// Example of an exact program to solve the Minimum Vertex Coloring Problem
// with GUROBI Solver. 
//
// Send comments/corrections to Flavio K. Miyazawa.
//----------------------------------------------------------------------
#include <gurobi_c++.h>
#include <float.h>
#include <math.h>
#include "mygraphlib.h"
#include "myutils.h"
#include <lemon/grosso_locatelli_pullan_mc.h>

using namespace lemon;

int main(int argc, char *argv[]) {
  Graph g;  // graph declaration
  string    filename;
  NodeStringMap  vname(g);  // name of graph nodes
  NodePosMap   px(g), py(g); // xy-coodinates for each node
  srand48(1);
  // uncomment one of these lines to change default pdf reader, or insert new one
  //set_pdfreader("open");    // pdf reader for Mac OS X
  //set_pdfreader("xpdf");    // pdf reader for Linux
  //set_pdfreader("evince");  // pdf reader for Linux

  if (argc!=2) {
    cout<< endl
	<< "This program computes a minimum vertex coloring in a graph\n" 
	<< "Usage: "<< argv[0]<<" <graph_filename>"<<endl << endl <<
      "Example:" << endl <<
      "\t" << argv[0] << " "<< getpath(argv[0])+"../instances/t_20.gr" << endl <<
      "\t" << argv[0] << " "<< getpath(argv[0])+"../instances/t_70.gr" << endl <<
      endl << endl   ;
    exit(0);}
  else if (!FileExists(argv[1])) {cout<<"File "<<argv[1]<<" does not exist."<<endl; exit(0);}

  filename = argv[1];
  string type = GetGraphFileType(filename);
  if (type!="graph"){cout<<"Error: Unknown type of graph: "<<type<<endl;exit(1);}
  GraphTable GT(filename,g); // Read the graph (only nodes and edges)
  GT.GetColumn("nodename",vname);
  GT.GetColumn("posx",px);
  GT.GetColumn("posy",py);

  GRBEnv env = GRBEnv();
  GRBModel model = GRBModel(env);
  model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE); 

  /* ILP variables */
  int ngreedycolors;
  cout << "Graph with " << countNodes(g) << " nodes and " << countEdges(g) << " edges." << endl;
  NodeIntMap cor(g); 
  GreedyColoring(g,ngreedycolors,cor);
  cout << "Greedy heuristic obtained coloring with "<<ngreedycolors<<" colors."<<endl;

  int maxcolorsILP=ngreedycolors-1; // For the ILP, we will try to obtain a coloring with less colors
  typedef vector<GRBVar> GRBVector;
  Graph::NodeMap<GRBVector> x(g,GRBVector(maxcolorsILP)); // x[v][i] indicates if node v has color i
  GRBVector y(maxcolorsILP); // y[i] indicates if color i is used.
  GRBLinExpr expressao;
  int totcolors;
  //model.getEnv().set(GRB_IntParam_Cuts, -1);
  model.getEnv().set(GRB_IntParam_CliqueCuts, 1);
  for (int i=0;i<maxcolorsILP;i++){
    char name[100];
    sprintf(name,"Y_%d",i+1);
    y[i] = model.addVar(0.0, 1.0, 1.0, GRB_BINARY,name); }
  
  for (NodeIt v(g); v != INVALID; ++v)
    for (int i=0;i<maxcolorsILP;i++){ char name[100]; sprintf(name,"X_%s_%d",vname[v].c_str(),i+1);
      x[v][i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY,name);  }
  model.update();

  for (NodeIt v(g); v != INVALID; ++v) { GRBLinExpr expr;  
    for (int i=0;i<maxcolorsILP;i++) expr += x[v][i];// Each node must have a color
    model.addConstr(expr == 1);  }

  for (EdgeIt e(g); e != INVALID; ++e) 
    for (int i=0;i<maxcolorsILP;i++)
      model.addConstr( x[g.u(e)][i] + x[g.v(e)][i] <= 1);// adjacent nodes must have different colors

  for (NodeIt v(g); v != INVALID; ++v)
    for (int i=0;i<maxcolorsILP;i++) 
      model.addConstr( x[v][i] <= y[i] );// if a node v has color i, then color i is used

  //------------------------------------------------------
  // We can also insert some improvement constraints 
  // 
  // Improving the formulation1: Removing some simmetry
  for (int i=1;i<maxcolorsILP;i++) model.addConstr( y[i-1] >= y[i] );

  if (1) {// Improving the formulatin2: pre-painting the nodes of a clique.
          // Try to see the impact if this improvement is not used (changing (1) to (0))
    //
    // If S={s0,s1,...} is a clique of G, the nodes of S must have different colors.
    // So, without loss of generality, we set s0 with color 0, s1 with color 1,...
    // Lemon graph library already has an heuristic to obtain a large clique
    GrossoLocatelliPullanMc<Graph> mc(g); 
    mc.iterationLimit(100);
    mc.run(GrossoLocatelliPullanMc<Graph>::DEGREE_BASED);
    if ((mc.cliqueSize()>=1)&&(mc.cliqueSize()<=countNodes(g))) {//obtained a clique
      Graph::NodeMap<bool> map(g); 
      mc.cliqueMap(map); // map indicate the nodes that are in the clique
      int i=0;
      for (NodeIt v(g); v != INVALID; ++v) { // if v is in the clique, it has color i
	if (map[v]) { model.addConstr( x[v][i] == 1 ); i++;}}}
  }
  try {
  //  model.write("model.lp"); system("cat model.lp");
    model.optimize();    // Insert cutting planes until we cannot separate the node x.
    if (model.get(GRB_IntAttr_SolCount) == 0)  // if could not obtain a solution
      throw "Could not obtain a solution.";
    totcolors = (int) floor(model.get(GRB_DoubleAttr_ObjVal)+0.001);
    for (NodeIt v(g); v!=INVALID; ++v)
      for (int i=0;i<totcolors;i++) if(BinaryIsOne(x[v][i].get(GRB_DoubleAttr_X))){ cor[v]=i;break; }
  } catch(GRBException e){
    cout << "Error number: " << e.getErrorCode() << endl;
    cout << e.getMessage() << endl;
  } catch (...) {
    cout << "Greedy heuristic obtained optimum coloring."<<endl;
    totcolors = ngreedycolors; 
    cout << "totcolors: " << totcolors << endl;
  }

  
  GraphAttributes G(g,vname,px,py);
  for (NodeIt v(g); v!=INVALID; ++v) G.SetLabel(v,vname[v] + "_" + IntToString(cor[v]+1));
  G.SetDefaultNodeAttrib("style=filled");
  for (EdgeIt e(g); e != INVALID; ++e) G.SetColor(e,"Black");

  if (totcolors<SizeVisualDistinctColor()){//try to use 'visual' colors(less options)
    for (NodeIt v(g); v!=INVALID; ++v)
      G.SetColor(v,ColorName(ith_VisualDistinctColor(cor[v])));
  }else {// there is no sufficient known colors
    for (NodeIt v(g); v!=INVALID; ++v) G.SetColor(v,"White"); }
  G.SetLabel("Vertices colored with "+IntToString(totcolors)+" colors (nodes with id and color)");
  for (NodeIt v(g);v!=INVALID;++v) cout << "color[" << vname[v] << "] = " << cor[v] << endl;
				     
  G.View();
  return(1);
}

