//
// Project and Analysis of Algorithms
// Flávio Keidi Miyazawa
//
// ex_viewgrahp.cpp
#include <stdio.h>
#include <lemon/list_graph.h>
#include "mygraphlib.h"
#include <string>
#include "myutils.h"
using namespace lemon;


int main(int argc, char *argv[]) 
{
  Digraph g;
  ArcValueMap weight(g);
  DNodeStringMap vname(g);
  ArcStringMap aname(g);
  DNodeColorMap vcolor(g);// color of nodes
  ArcColorMap acolor(g); // color of edges
  DNodePosMap   posx(g),posy(g);
  string filename,graphtitle;


  // uncomment one of these lines to change default pdf reader, or insert new one
  //set_pdfreader("open");    // pdf reader for Mac OS X
  //set_pdfreader("xpdf");    // pdf reader for Linux
  //set_pdfreader("evince");  // pdf reader for Linux

  if (argc!=2) {cout<< endl << "Usage: "<< argv[0]<<" <graph_filename>"<<endl << endl <<
      "Example:" << endl <<
      "\t" << argv[0] << " "<< getpath(argv[0])+"../instances/t_100.dig" << endl <<
      endl << endl; exit(0);}
  
  else if (!FileExists(argv[1])) {cout<<"File "<<argv[1]<<" does not exist."<<endl; exit(0);}
  filename = argv[1];
  
  // Read the graph
  if (!ReadDigraph(filename,g,vname,posx,posy,weight))
    {cout<<"Error reading graph file "<<argv[1]<<"."<<endl;exit(0);}

  // Print some information of the graph
  graphtitle = "Digraph with " + IntToString(countNodes(g)) + " nodes and " + IntToString(countArcs(g)) + " arcs.";
  cout << graphtitle << "\n"; 
  cout << "List of Nodes\n";
  for (DNodeIt v(g);v!=INVALID;++v)
    cout<<vname[v]<<" xy-pos("<<posx[v]<<" , "<<posy[v]<<")\n";
  cout << "\n";

  cout << "List of Arcs\n";
  for (ArcIt a(g); a!=INVALID; ++a) 
    cout<<vname[g.source(a)]+" --> "+vname[g.target(a)]<<"  weight="<<weight[a]<<"\n"; 
  cout << "\n";

  // Prepare and view the graph
  DigraphAttributes G(g,vname,posx,posy);
  G.SetDefaultDNodeAttrib("color=Gray style=filled");
  G.SetLabel(graphtitle);
  G.View();
  
  return(1);
}

