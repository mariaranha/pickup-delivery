#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include "mygraphlib.h"
using namespace std;
int main(int argc, char *argv[]) 
{
  int n;
  srand48(clock());
  if (argc!=5) {cout<<"Usage: "<< argv[0]<<" <number_of_nodes> <edge probability> <lb edgeweight> <ub edgeweight>"<<endl; exit(0);} 
  n = atoi(argv[1]);
  Graph g;
  NodeStringMap vname(g);
  NodePosMap px(g);
  NodePosMap py(g);
  EdgeValueMap weight(g);
  double p,lb,ub;
  p = atof(argv[2]);
  lb = atof(argv[3]);
  ub = atof(argv[4]);
  GenerateRandomGraph(g,n,vname,px,py,p,lb,ub,weight,10000,10000);
  cout << "nnodes nedges type" << endl;
  cout << countNodes(g) << " " << countEdges(g) << " graph" << endl;
  cout << "nodename posx posy" << endl;
  for (NodeIt v(g);v!=INVALID;++v) 
    cout<<vname[v]<<" "<<px[v]<<" "<<py[v]<<endl;
  cout << "endpoint1 endpoint2 weight" << endl;
  for (EdgeIt e(g);e!=INVALID;++e)
    cout<<vname[g.u(e)]<<" "<<vname[g.v(e)]<<" "<<weight[e]<<endl;
}
