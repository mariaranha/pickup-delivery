// Project and Analysis of Algorithms
// Flávio Keidi Miyazawa
// Problems with connectivity: Generate Triangulated Graph 
#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <lemon/list_graph.h>
#include "mygraphlib.h"
#include <string>
#include "myutils.h"
using namespace lemon;


int main(int argc, char *argv[]) 
{
  int n;
  double box_width,box_height;
  Graph g;  // graph declaration
  NodeStringMap vname(g);  // name of graph nodes
  NodePosMap px(g),py(g);  // xy-coodinates for each node
  NodeColorMap vcolor(g);// color of nodes
  EdgeColorMap ecolor(g); // color of edges
  EdgeValueMap weight(g);   // edge weights
  vector <Node> V;
  srand48(1);

  // double cutoff;   // used to prune non promissing branches (of the B&B tree)
  if (argc!=4) {cout<<"Usage: "<< argv[0]<<"<number_of_nodes_in_graph> <box_width> <box_height>"<< endl;exit(0);}

  n = atoi(argv[1]);
  box_width = atof(argv[2]);
  box_height = atof(argv[3]);

  GenerateTriangulatedGraph(g,vname,px,py,weight,n,box_width,box_height);

  cout << endl;
  cout << "# Graph generated with the command:" << endl;
  cout << "# " << argv[0] << " " << argv[1] << " " << argv[2] << " " << argv[3] << endl;
  cout << endl;
  cout << "# In the first table, we have the graph informations:" << endl;
  cout << "# nnodes    - Number of nodes." << endl;
  cout << "# nedges    - Number of edges." << endl;
  cout << "# type      - graph - Graph given by list of edges." << endl;
  cout << endl;
  cout << "# The second table is the list of <nnodes> nodes, with columns:" << endl;
  cout << "# nodename  - Name of the node (without spaces)." << endl;
  cout << "# posx posy - xy-location of the node." << endl;
  cout << endl;
  cout << "# The third table is the list of <nedges> edges, with columns:" << endl;
  cout << "# node1 node2 - endpoints of the edge." << endl;
  cout << "# weight      - cost of the edge {node1,node2}." << endl;
  cout << endl;
  cout << "nnodes\tnedges\ttype" << endl;
  cout << countNodes(g) << "\t" << countEdges(g) << "\tgraph" << endl << endl;;
  cout << "nodename\tposx\tposy" << endl;
  for (NodeIt v(g);v!=INVALID;++v) 
    cout << vname[v] << " " << px[v] << " " << py[v] << endl ;
  cout << endl;
  cout << "endpoint1" << "\t" << "endpoint2" << "\t" << "weight" << endl;
  for (EdgeIt e(g);e!=INVALID;++e) 
    cout << vname[g.u(e)] << " " << vname[g.v(e)] << " " << weight[e] << endl;
  return 0;
}

