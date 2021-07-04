
// Project and Analysis of Algorithms
// Flávio Keidi Miyazawa
// Problems with connectivity: Generate Triangulated Digraph 
#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <lemon/list_graph.h>
#include "mygraphlib.h"
#include <string>
#include "myutils.h"
#include <lemon/concepts/digraph.h>
#include <lemon/preflow.h>
using namespace lemon;


int main(int argc, char *argv[]) 
{
  int n;
  double box_width,box_height;
  Digraph g;  // graph declaration
  DNodeStringMap vname(g);  // name of graph nodes
  DNodePosMap px(g),py(g);  // xy-coodinates for each node
  DNodeColorMap vcolor(g);// color of nodes
  ArcColorMap ecolor(g); // color of edges
  ArcValueMap weight(g);   // edge weights
  vector <DNode> V;
  srand48(1);

  // double cutoff;   // used to prune non promissing branches (of the B&B tree)
  if (argc!=4) {cout<<"Usage: "<< argv[0]<<" <number_of_nodes_in_graph> <box_width> <box_height>"<< endl;exit(0);}

  n = atoi(argv[1]);
  box_width = atof(argv[2]);
  box_height = atof(argv[3]);

  GenerateTriangulatedDigraph(g,vname,px,py,weight,n,box_width,box_height);
  cout << "nnodes narcs type" << endl;
  cout << countNodes(g) << " " << countArcs(g) << " digraph" << endl;
  cout << "nodename posx posy" << endl;
  for (DNodeIt v(g);v!=INVALID;++v) cout<<vname[v]<<" "<<px[v]<<" "<<py[v]<< endl;
  cout << "tail head weight" << endl;
  for (ArcIt e(g);e!=INVALID;++e) 
    cout << vname[g.source(e)] << " " << vname[g.target(e)] << " " << weight[e] << endl;
  return 0;
}

