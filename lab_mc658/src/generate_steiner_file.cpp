// Project and Analysis of Algorithms
// Flávio Keidi Miyazawa
// Problems with connectivity: Minimum Cost Steiner Tree
#include <lemon/list_graph.h>
#include "mygraphlib.h"
#include <string>
//#include <lemon/concepts/digraph.h>
#include <lemon/preflow.h>
using namespace lemon;



int main(int argc, char *argv[]) 
{
  int nt;
  Graph g;  // graph declaration
  string graph_filename, terminals_filename;
  NodeStringMap vname(g);  // name of graph nodes
  NodeColorMap vcolor(g);// color of nodes
  NodePosMap posx(g); // coord. x dos vertices
  NodePosMap posy(g); // coord. y dos vertices
  NodeIntMap is_terminal(g);// binary variable, indicate is node is terminal
  EdgeColorMap ecolor(g); // color of edges
  EdgeValueMap weight(g);   // edge weights
  srand48(1);

  if (argc!=3) {
    cout<<"Usage: "<< argv[0]<<" <graph_filename> <number_of_random_terminals>"<<endl;
    exit(0);}
  
  graph_filename = argv[1];
  stringstream(argv[2]) >> nt; // number of terminals

  // Generate a random euclidean graph with n=500 points in the region [0,100)x[0,100)
  // GenerateTriangulatedDigraph(g,vname,px,py,weight,500,100,100);
  // GenerateRandomEuclideanDigraph(g,vname,px,py,weight,n,100,100);
  ReadGraph(graph_filename,g,vname,posx,posy,weight);
  int n = countNodes(g) ;
  int m = countEdges(g) ;
  vector <Node> V; // Node V[0] is the root. Nodes V[1], ... , V[nterminals-1] are the destination
  for (NodeIt v(g); v!=INVALID; ++v) {V.push_back(v);is_terminal[v]=0;}
  // Perform a permutation on the nodes. The first nt nodes are the terminals.
  for (int i=0;i<n;i++) {Node aux;int j=(int)(n*drand48()); aux = V[j]; V[j] = V[i];V[i] = aux;}
  for (int i=0;i<nt;i++) {is_terminal[V[i]]=1;}

  cout << "nnodes nedges type" << endl; 
  cout << n << " " <<  m << " graph" << endl;
  cout << "nodename posx posy terminal" << endl;
  for (NodeIt v(g); v!=INVALID; ++v) 
    cout << vname[v] << " " << posx[v] << " " << posy[v] << " " << is_terminal[v] << endl;
  cout << "endpoint1 endpoint2 weight" << endl;
  for (EdgeIt e(g); e!=INVALID; ++e) 
    cout << vname[g.u(e)] << " " <<vname[g.v(e)] << " " << weight[e] << endl;
  return 0;
}

