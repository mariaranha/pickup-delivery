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
  Graph g;
  EdgeValueMap weight(g);
  NodeStringMap vname(g);
  EdgeStringMap ename(g);
  NodeColorMap vcolor(g);// color of nodes
  EdgeColorMap ecolor(g); // color of edges
  NodePosMap   posx(g),posy(g);
  string filename,graphtitle;


  // uncomment one of these lines to change default pdf reader, or insert new one
  //set_pdfreader("open");    // pdf reader for Mac OS X
  //set_pdfreader("xpdf");    // pdf reader for Linux
  //set_pdfreader("evince");  // pdf reader for Linux

  if (argc!=2) {cout<< endl << "Usage: "<< argv[0]<<" <graph_filename>"<<endl << endl <<
      "Example:" << endl <<
      "\t" << argv[0] << " "<< getpath(argv[0])+"../instances/s_50.gr" << endl <<
      "\t" << argv[0] << " "<< getpath(argv[0])+"../instances/t_100.gr" << endl << endl; exit(0);}
  
  else if (!FileExists(argv[1])) {cout<<"File "<<argv[1]<<" does not exist."<<endl; exit(0);}
  filename = argv[1];
  
  // Read the graph
  if (!ReadGraph(filename,g,vname,posx,posy,weight)) 
    {cout<<"Error reading graph file "<<argv[1]<<"."<<endl;exit(0);}

  graphtitle = "Graph '" + filename + "' with " + IntToString(countNodes(g)) + " nodes and " + IntToString(countEdges(g)) + " edges.";

    // Prepare and view the graph
  GraphAttributes G(g,vname,posx,posy);
  G.SetDefaultNodeAttrib("color=Gray style=filled");
  G.SetLabel(weight);
  G.SetLabel(graphtitle);
  G.View();

  return(1);
}

