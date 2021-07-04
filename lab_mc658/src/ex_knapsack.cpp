//----------------------------------------------------------------------
// Example of an exact program to solve the knapsack problem
// with GUROBI Solver.
//
// Send comments/corrections to Flavio K. Miyazawa.
//----------------------------------------------------------------------
#include <gurobi_c++.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <sstream>
#include <fstream>
using namespace std;

int main() 
{
  int time_limit,n;
  n = 100;
  vector<double> size(n);
  vector<double> value(n);
  double Capacity,total_value,total_size;

  srand(1);
  time_limit = 3600; // solution must be obtained within time_limit seconds
  Capacity = 100;

  vector<GRBVar> x(n);
  GRBEnv env = GRBEnv();
  GRBModel model = GRBModel(env);
  GRBLinExpr expr;
  model.set(GRB_StringAttr_ModelName, "Knapsack Example"); // gives a name to the problem
  model.set(GRB_IntAttr_ModelSense, GRB_MAXIMIZE); // says that lp is a maximization problem
  
  for (int i=0;i<n;i++) {
    size[i] = (double) ((int) (drand48()*Capacity));
    value[i] = (double) ((int) (drand48()*Capacity)/10.0);
    x[i] = model.addVar(0.0, 1.0, value[i],GRB_BINARY,"");
    expr += size[i]*x[i];
  }
  model.update(); // run update to use model inserted variables
  model.addConstr(expr <= Capacity);

  //-----------------------
  cout << "0/1 Knapsack Problem:" << endl; 
  cout << "Number of items: " << n << endl; 
  cout << "Capacity       : " << Capacity << endl; 
  for (int i=0;i<n;i++) cout << "Item_" << i+1 << ": \tSize:\t" << size[i] << "\tValue\t" << value[i] << endl;
  //-----------------------
  
  try {// bound the execution time 
    if (time_limit > 0) model.getEnv().set(GRB_DoubleParam_TimeLimit,time_limit);
    model.optimize();
    if (model.get(GRB_IntAttr_SolCount) == 0)  // if could not obtain a solution
      throw "Could not obtain a solution.";
    total_value = 0.0;  total_size = 0.0;
  //-----------------------
  cout << "Solution:: " << endl; 

  for (int i=0;i<n;i++) {
    if (x[i].get(GRB_DoubleAttr_X) > 0.999) {
      total_value += value[i];  total_size += size[i];
      cout<<"Item_"<<i+1<<": \tSize:\t"<<size[i]<<"\tValue\t"<<value[i]<<endl;}}
  cout<<endl<<"Capacity    = "<<Capacity<<endl<<"Total size  = "<<total_size<<endl;
  cout << "Total value = "<< total_value << endl << endl;
  return 0;
  }catch (...) { printf("Exception...\n");  exit(1); }
}
