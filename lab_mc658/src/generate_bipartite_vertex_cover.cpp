#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include "mygraphlib.h"
using namespace std;

// Gera um grafo bipartido completo
int main(int argc, char *argv[]) 
{
  int n,m;
  srand48(clock());
  if (argc!=4) {cout<<"Usage: "<< argv[0]<<" <#nodes in A>  <#nodes in B>  <edge_density>"<<endl; exit(0);} 
  n = atoi(argv[1]);
  m = atoi(argv[2]);
  double d = atof(argv[3]);
  if ((d<=0)||(d>1)) {cout << "Error: <edge_density> must be in the interval (0,1]." <<endl;exit(0);}
  
  Graph g;
  vector <Node> A;
  vector <Node> B;
  NodeStringMap vname(g);
  NodeValueMap weight(g);
  NodePosMap px(g),py(g);
  A.reserve(n);A.resize(n);
  B.reserve(m);B.resize(m);
  for (int i=0;i<n;i++) {
    A[i]=g.addNode();
    px[A[i]] = 2000*(double) i/(n-1);
    py[A[i]] = 1000;
    vname[A[i]] = "A"+IntToString(i+1);}
  for (int i=0;i<m;i++) {
    B[i]=g.addNode();
    px[B[i]] = 2000*(double) i/(n-1);
    py[B[i]] = 10;
    vname[B[i]] = "B"+IntToString(i+1);}

  for (int i=0;i<n;i++) {
    for (int j=0;j<m;j++) {
      if (drand48()<=d) g.addEdge(A[i],B[j]); }}

  for (NodeIt v(g);v!=INVALID;++v) weight[v] = (int) (100*drand48()+1);

  // print the graph
  cout << "nnodes nedges type" << endl;
  cout << countNodes(g) << " " << countEdges(g) << " graph" << endl;
  cout << "nodename posx posy part weight" << endl;
  for (int i=0;i<n;i++) 
    cout<<vname[A[i]]<<"  "<<px[A[i]]<<" "<<py[A[i]]<<" 0 "<<weight[A[i]]<<endl;
  for (int i=0;i<m;i++) 
    cout<<vname[B[i]]<<"  "<<px[B[i]]<<" "<<py[B[i]]<<" 1 "<<weight[B[i]]<<endl;
  cout << "endpoint1 endpoint2" << endl;
  for (EdgeIt e(g);e!=INVALID;++e)
    cout << vname[g.u(e)]<<" "<<vname[g.v(e)]<< endl;
}
