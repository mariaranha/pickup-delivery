
//----------------------------------------------------------------------
// Example of the use of the greedy algorithm for the minimum vertex
// coloring problem
//
// Send comments/corrections to Flavio K. Miyazawa.
//----------------------------------------------------------------------
#include "mygraphlib.h"
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

  int ncolors; NodeIntMap nodecolor(g);
  GreedyColoring(g,ncolors,nodecolor);
  cout << "Greedy coloring used " << ncolors<< " colors."<<endl;
  for (NodeIt v(g);v!=INVALID;++v) vname[v] = "\""+vname[v]+"_"+IntToString(nodecolor[v]+1)+"\"";

  // Verify if the coloring is proper
  for (EdgeIt e(g);e!=INVALID;++e) {
    if (nodecolor[g.u(e)]==nodecolor[g.v(e)]) {
      cout<<"Error: Edge conecting nodes "<<vname[g.u(e)]<<" and "<<vname[g.v(e)]<<" has same color\n";
      getchar(); }}

  // Visualize the graph colored, if possible (otherwise, the color is given after the name of each node)
  GraphAttributes G(g,vname,px,py);
  G.SetDefaultNodeAttrib("style=filled");
  if (ncolors<SizeVisualDistinctColor()){//try to use 'visual' colors(less options)
    for (NodeIt v(g); v!=INVALID; ++v)
      G.SetColor(v,ColorName(ith_VisualDistinctColor(nodecolor[v])));}
  return(G.View());
  
}

