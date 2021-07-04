#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include "mygraphlib.h"
using namespace std;

// Gera um grafo bipartido completo
int main(int argc, char *argv[]) 
{
  int n,m,x[1000],y[1000];
  srand48(clock());
  if (argc!=3) {cout<<"Usage: "<< argv[0]<<"   <#nodes in A>  "<< "  <#nodes in B>"<<endl; exit(0);} 
  n = atoi(argv[1]);
  m = atoi(argv[2]);
  cout << n+m << " " << n*m << endl; // number of nodes and number of edges (complete bipartite)
  // gera os pontos na parte A
  for (int i=0;i<n;i++) {
    x[i] = ((int) (drand48()*1000)+1); 
    y[i] = ((int) (drand48()*1000)+3000) ;
    cout << i+1 << " " <<  x[i] << " " << y[i] << endl;
  }
  // gera os pontos na parte B
  for (int i=n;i<n+m;i++) {
    x[i] = ((int) (drand48()*1000)+1);
    y[i] = ((int) (drand48()*1000)+1);
    cout << i+1 << " " <<  x[i] << " " << y[i] << endl;
  }
  // gera as arestas, formando grafo bipartido completo
  for (int i=0;i<n;i++) {
    for (int j=n;j<n+m;j++) {
      cout << i+1 << " " <<  j+1 << endl;
    }
  }
}
