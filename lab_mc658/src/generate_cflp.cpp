#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <fstream>
using namespace std;

// Gera um grafo bipartido completo
int main(int argc, char *argv[]) 
{
  int nf,nc,m,totcapacity,totdemand;
  double density,openfactor;
  srand48(clock());
  if (argc!=5) {cout<<"Usage: "<< argv[0]<<"   <#facilities>  "<< "  <#clients>"<<" <density in (0,1)> <fac_open_factor>" << endl;
    cout << "Example:" << endl;
    cout << "\t" << argv[0] << " 10 20 .5 10" << endl;
    exit(0);} 
  nf = atoi(argv[1]); nf++;
  nc = atoi(argv[2]);
  density = atof(argv[3]);
  openfactor = atof(argv[4]);
  m = (nf*nc-nf-nc)*density; if (m<0) m=0;
  vector<vector<int> > cost;
  cost.reserve(nf); cost.resize(nf);
  for(int i=0;i<nf;i++) {cost[i].reserve(nc);cost[i].resize(nc);}
  vector<double> capacity; capacity.reserve(nf);capacity.resize(nf);
  vector<double> opencost; opencost.reserve(nf);opencost.resize(nf);
  vector<double> demand; demand.reserve(nc);demand.resize(nc);
  int costmagnitude;
  double maxcapacity=20;
  totcapacity = 0; totdemand=0;
  for(int i=1;i<nf;i++) {
    for(int j=0;j<nc;j++) cost[i][j] = 0; // 0 means that there is no edge
    capacity[i] = (int) (drand48()*maxcapacity) + 1; totcapacity+=capacity[i];
    opencost[i] = (int) (drand48()*openfactor) + 1;
  } // capacity is a number in [1,5]
  for(int j=0;j<nc;j++) {
    demand[j] = (int) (drand48()*maxcapacity/5) + 1; // all client demands are 1 (you can choose another positive value)
    totdemand += demand[j];}

  costmagnitude = 9;
  // All instances are feasible (can be connected to facility 0),
  // but each connection cost to 0 will be very expensive.
  capacity[0] = totdemand;
  opencost[0] = 0.00001; // small open cost
  for(int j=0;j<nc;j++) cost[0][j] = 10000000; // very large connection cost

  // each facility has at least one client
  for(int i=1;i<nf;i++) cost[i][(int) (drand48()*nc)] = (int) (drand48()*costmagnitude)+1;
  // each client has at least one facility (between facility 1 and facility nf-1)
  for(int j=0;j<nc;j++) cost[(int) (drand48()*(nf-1))+1][j] = (int) (drand48()*costmagnitude)+1;
  for (int k=0;k<m;k++)
    cost[(int) (drand48()*(nf-1))+1][(int) (drand48()*nc)] = (int) (drand48()*costmagnitude)+1;
  m=0; // count number of real edges
  for(int i=0;i<nf;i++) for(int j=0;j<nc;j++) if (cost[i][j]>0) m++;

  // Print the instance
  cout << "# This instance has 4 tables: " << endl;
  cout << "# The first table, with one line, has the number of lines in the" << endl;
  cout << "# next three tables. I.e., The #facilities, #clients " << endl;
  cout << "# and #connections/edges between facilities and clients" << endl;
  cout << "#" << endl;
  cout << "# The second table has facility informations. Each facility has capacity and open cost values." << endl;
  cout << "# It will generate one more facility, f0, that is connected to all clients." << endl;
  cout << "# With this, all instances will be feasible, as the facility f0 will have sufficient capacity" << endl;
  cout << "# but connections to f0 are avoided, because it has very large connection cost to each client." << endl;
  cout << "#" << endl;
  cout << "# The third table has client informations. Each client has a demand." << endl;
  cout << "#" << endl;
  cout << "# The fourth table has the connection costs between a facility and client." << endl;
  cout << endl;

  
  // table with one line, containing numbers of facilities and clients
  cout << "nfacilities nclients nconnectioncosts" << endl;
  cout << nf << "\t" << nc << "\t" << m << endl;

  // table of facilities
  cout << "facility capacity opencost" << endl;
  for(int i=0;i<nf;i++) cout<<"f"<<i<<"\t"<<capacity[i]<<"\t"<<opencost[i]<<endl;

  // table of clients
  cout << "client demand" << endl;
  for(int j=0;j<nc;j++)  cout << "c" << j+1 << "\t" << demand[j] << endl;

  // table of connection costs
  cout << "facility client connectioncost" << endl;
  for(int i=0;i<nf;i++)
    for(int j=0;j<nc;j++)
      if (cost[i][j]>0)
	cout<<"f"<<i<<"\t"<<"c"<<j+1<<"\t"<<cost[i][j] << endl;
  
}
