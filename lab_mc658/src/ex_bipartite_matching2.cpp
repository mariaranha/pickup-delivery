// Project and Analysis of Algorithms
// Flávio Keidi Miyazawa
// Problems with connectivity: Maximum Bipartite Matching, without graph data structure
#include <stdio.h>
#include <string>
#include "myutils.h"
#include <iomanip>      // std::setprecision
#include <iostream>     // std::cout, std::fixed
#include <gurobi_c++.h>
using namespace std;

int main(int argc, char *argv[]) 
{
  int nA,nB;
  srand48(1);
  if (argc!=3) {
    cout << "Find a maximum  weighted matching  in a  bipartite\n";
    cout << "graph G=(A,B,E), where (A,B) is a partition of the\n";
    cout << "graph nodes E is the set of edges in G, each  edge\n";
    cout << "{a,b} in E connects a node a in A to a node b in B.\n";
    cout << endl;
    cout << "Usage: "<< argv[0]<<" <#nodes_in_A>"<<" <#nodes_in_B>"<<endl;
      cout<<"       Edge weights are (pseudo) random numbers in [0,100)," << endl;
    exit(0);
  }
  nA = atoi(argv[1]);  nB = atoi(argv[2]);
  if ((nA<1) || (nA>10000) || (nB<1) || (nB>10000)) {
    cout << "Invalid or too large number of nodes.\n"; exit(0);  }
  GRBEnv env = GRBEnv();  GRBModel model = GRBModel(env);

  double E[nA][nB];
  vector <vector<GRBVar> >x(nA, vector<GRBVar>(nB)); // a matrix x[nA][nB] 
  model.set(GRB_StringAttr_ModelName, "Maximum Bipartite Matching"); // formulation name
  model.set(GRB_IntAttr_ModelSense, GRB_MAXIMIZE); // objective sense
  
  // Create/Insert variables of the linear program  X_i_j, with i in A and j in B
  for (int i=0;i<nA;i++) {
    for (int j=0;j<nB;j++) {
      E[i][j]=(int) (drand48()*100.0);
      x[i][j]=model.addVar(0.0,1.0,E[i][j],GRB_CONTINUOUS,
			   "X_"+IntToString(i)+"_"+IntToString(i));  } }
  
  model.update();

  // for each node i in X, we can only select one edge incident to i
  for (int i=0;i<nA;i++) { GRBLinExpr S;
    for (int j=0;j<nB;j++) S += x[i][j];
    model.addConstr(S  <= 1 );  }
  
  // for each node j in Y, we can only select one edge incident to i
  for (int j=0;j<nB;j++) {  GRBLinExpr S;
    for (int i=0;i<nA;i++) S += x[i][j];
    model.addConstr(S  <= 1 );  }
  
  try {
    model.optimize(); 
    double sum=0.0;

    cout << "\nSolution X obtained from the Linear Program (without integrality constraints)" << endl << "    ";
    for (int j=0;j<nB;j++) cout << setw(5) << " B"+IntToString(j) << "  ";
    cout << endl;
    for (int i=0;i<nA;i++) {
      cout << "A" << i << "| ";
      for (int j=0;j<nB;j++) 
	if (x[i][j].get(GRB_DoubleAttr_X)>0.000001)
	  cout << setw(5) <<  "*"+DoubleToString(x[i][j].get(GRB_DoubleAttr_X)) << "* ";
	else
	  cout << setw(5) <<  " "+DoubleToString(x[i][j].get(GRB_DoubleAttr_X)) << "  ";
      cout << endl;   }

    cout << endl << endl << "Edge Weights" << endl << "    ";
    for (int j=0;j<nB;j++) cout << setw(5) << " B"+IntToString(j) << "  ";
    cout << endl;
    for (int i=0;i<nA;i++) {
      cout << "A" << i << "| ";
      for (int j=0;j<nB;j++) {
	if (x[i][j].get(GRB_DoubleAttr_X)>0.999)
	  cout << setw(5) <<  "*"+IntToString((int) E[i][j]) << "* ";
	else
	  cout << setw(5) <<  " "+IntToString((int) E[i][j]) << "  ";  }
      cout << endl;  }

    cout << endl << endl << "Edges in the solution:" << endl;
    for (int i=0;i<nA;i++) {
      for (int j=0;j<nB;j++) {
	if (x[i][j].get(GRB_DoubleAttr_X)>0.999)  {
	  cout <<  "Edge " << setprecision(3) << "A"+IntToString(i) << "-----" <<  "B"+IntToString(j) << ":  " << E[i][j] << endl;
	  sum += E[i][j];
	  break; } } }
    cout << "Sum of edges: "<< sum << " (must be equal to "
	 << model.get(GRB_DoubleAttr_ObjVal) << ", obtained from objective function)\n" << endl;
  } catch(GRBException e) {// A problem had occurred...
    cerr << "Could not solve the linear program formulation." << endl;
    cerr << "Error code: " << e.getErrorCode() << endl;
    cerr << e.getMessage(); }
  return 0;
}

