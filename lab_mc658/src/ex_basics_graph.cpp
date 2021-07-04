//----------------------------------------------------------------------
// Some basic routines using non-oriented graphs
// Send comments/corrections to Flavio K. Miyazawa.
//----------------------------------------------------------------------
//#include <gurobi_c++.h>
#include <float.h>
#include <math.h>
#include <set>
#include <lemon/list_graph.h>
#include <lemon/gomory_hu.h>
#include "mygraphlib.h"
#include "myutils.h"

int main(int argc, char *argv[]) 
{
  Graph g;
  EdgeValueMap weight(g);
  NodeStringMap vname(g);
  EdgeStringMap ename(g);
  NodePosMap   posx(g),posy(g);
  string filename;

  // uncomment one of these lines to change default pdf reader, or insert new one
  set_pdfreader("open");    // pdf reader for Mac OS X
  //set_pdfreader("xpdf");    // pdf reader for Linux
  //set_pdfreader("evince");  // pdf reader for Linux

  if (argc!=2) {cout<< endl << "Usage: "<< argv[0]<<" <graph_filename>"<<endl << endl <<
      "Example:" << endl <<
      "\t" << argv[0] << " "<< getpath(argv[0])+"../instances/g_7.gr" << endl <<
      "\t" << argv[0] << " "<< getpath(argv[0])+"../instances/t_70.gr" << endl << endl; exit(0);}
  
  else if (!FileExists(argv[1])) {cout<<"File "<<argv[1]<<" does not exist."<<endl; exit(0);}
  filename = argv[1];

  // Read the graph
  if (!ReadGraph(filename,g,vname,posx,posy,weight)) 
    {cout<<"Error reading graph file "<<argv[1]<<"."<<endl;exit(0);}

  cout << "List of Nodes\n";
  for (NodeIt v(g); v!=INVALID; ++v)
    cout << vname[v] << "  ";
  cout << "\n==============================================================\n\n";

  cout << "List of Edges\n";
  for (EdgeIt e(g); e!=INVALID; ++e) {
    ename[e] = "{"+vname[g.u(e)]+","+vname[g.v(e)]+"}_"+DoubleToString(weight[e]);
    cout << ename[e] << "  ";
  }
  cout << "\n==============================================================\n\n";


  cout << "List of Edges incident to nodes\n";
  for (NodeIt v(g); v!=INVALID; ++v) {
    cout << "Node " << vname[v] << ": ";
    for (IncEdgeIt e(g,v); e!=INVALID; ++e) cout << ename[e] << "  ";
    cout << "\n\n";
  }
  cout << "==============================================================\n\n";

  cout <<  filename << "\n";

  // View the graph
  GraphAttributes G(g,vname,posx,posy);
  G.SetDefaultNodeAttrib("color=Red");
  G.SetDefaultEdgeAttrib("color=Blue");
  G.SetLabel("Grafo '"+filename+"' com "+IntToString(countNodes(g))+" vertices e "+IntToString(countEdges(g))+" arestas.");
  G.View();
}
