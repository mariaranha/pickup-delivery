//
// Project and Analysis of Algorithms
// Flávio Keidi Miyazawa
//
// This is only to view the shortest path/tree obtained by the Lemon graph library
//
#include <stdio.h>
#include <lemon/list_graph.h>
#include "mygraphlib.h"
#include "myutils.h"
#include <string>
#include <vector>
#include <time.h>
#include <math.h>
#include <sstream>
#include <iostream>  
#include <fstream>
#include <iomanip>
#include <lemon/path.h>
#include <lemon/dijkstra.h>
using namespace std;

int main(int argc, char *argv[]) 
{
  Digraph g;
  ArcValueMap weight(g);
  DNodeStringMap vname(g);
  DNodePosMap   posx(g),posy(g);
  string filename,graphtitle,source_node_name,target_node_name;
  DNode sourcenode,targetnode;
  // uncomment one of these lines to change default pdf reader, or insert new one
  //set_pdfreader("open");    // pdf reader for Mac OS X
  //set_pdfreader("xpdf");    // pdf reader for Linux
  //set_pdfreader("evince");  // pdf reader for Linux
  if (argc!=4) {cout<<endl<<
      "Program to obtain a shortest path from node s to a node t," << endl <<
      "Usage: "<< argv[0]<<"  <graph>  <source_node_name>  <target_node_name>"<<
      endl << endl;
    cout << "Example:" << endl <<
      "\t"<<argv[0]<<" "<<getpath(argv[0])+"../instances/t_100.dig 12 50"<<endl<<endl;
    exit(0);}

  filename = argv[1];   source_node_name = argv[2];  target_node_name = argv[3];
  
  if (!ReadDigraph(filename,g,vname,posx,posy,weight))   // Read the graph
    {cout<<"Error reading digraph file "<<argv[1]<<"."<<endl;exit(0);}

  bool found=false; // Find the source node in the digraph
  for (DNodeIt v(g);v!=INVALID;++v)
    if(vname[v]==source_node_name){sourcenode=v;found=true; break;}
  if (!found){cout<<"Could not find source node \""<<source_node_name<<"\""<<endl;
    exit(0);}

  found=false; // Find the target node in the digraph
  for (DNodeIt v(g);v!=INVALID;++v)
    if(vname[v]==target_node_name){targetnode=v;found=true;break;}
  if (!found){cout<<"Could not find target node "<<target_node_name<<endl;
    exit(0);}

  auto start_time = chrono::high_resolution_clock::now();
  Dijkstra<Digraph, ArcValueMap>  Dij(g,weight); Dij.run(sourcenode,targetnode);
  auto end_time = chrono::high_resolution_clock::now();
  auto time = end_time - start_time;
  cout << "Running Time in microseconds: " <<
    chrono::duration_cast<chrono::microseconds>(time).count() << endl;
  cout << "Shortest path cost: " << Dij.dist(targetnode)<<endl;

  DigraphAttributes G(g,vname,posx,posy);
  //G.SetDigraphAttrib("fontsize=16");
  G.SetDefaultDNodeAttrib("color=Gray style=filled width=0.3 height=0.3 fixedsize=true");
  G.SetDefaultArcAttrib("color=Gray");
  // G.SetLabel(weight);  // figure becomes with too many numbers...

  G.SetColor(sourcenode,"Red");  G.SetColor(targetnode,"Cyan");

  DNode v = targetnode; // paint the shortest-path tree with color blue
  for (DNodeIt v(g);v!=INVALID;++v) G.SetColor(Dij.predArc(v),"Blue");

  // paint the shortest-path from source to target with color blue
  while (v!=sourcenode) { G.SetColor(Dij.predArc(v),"Red"); v = Dij.predNode(v); }
  G.SetLabel("Shortest path from "+vname[sourcenode]+" to "+vname[targetnode]+
	     ", in red, have distance "+DoubleToString(Dij.dist(targetnode))+
	     ". The shortest-path tree is given in blue/red.");
  G.View();
  return(1);
}

