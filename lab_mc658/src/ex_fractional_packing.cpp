//----------------------------------------------------------------------
// Example of an exact program to solve relaxation of a packing problem
// with GUROBI Solver. See pages 105-108, 124-125 of 
// http://www.ic.unicamp.br/~fkm/lectures/proglin.pdf  (in portuguese)
// Send comments/corrections to Flavio K. Miyazawa.
//----------------------------------------------------------------------
#include <iostream>
using namespace std;
#include <gurobi_c++.h>
double fractional_packing() 
{
  int n=4;
  vector<GRBVar> x(n);
  GRBEnv env = GRBEnv();
  GRBModel model = GRBModel(env);
  cout << endl << endl <<"Relaxation of the integer linear program in page 124 of the slides in " << endl <<
    "http://www.ic.unicamp.br/~fkm/lectures/proglin.pdf" << endl << endl << endl;

  for (int i=0;i<n;i++) x[i]=model.addVar(0.0,GRB_INFINITY,1.0,GRB_CONTINUOUS,"");
  model.update(); // run update to use model inserted variables
  model.addConstr(3*x[0] + 8*x[1] +          2*x[3] >= 950);
  model.addConstr(2*x[0] +            x[2] + 3*x[3] >= 1150);
  model.addConstr(  x[0] +          3*x[2] +   x[3] >= 495);
  model.addConstr(  x[0] +          2*x[2] +   x[3] >= 450);

  model.set(GRB_StringAttr_ModelName, "Packing Example"); 
  model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
  try{
    model.optimize();
    if (model.get(GRB_IntAttr_SolCount) == 0)  // if could not obtain a solution
      throw "Could not obtain a solution.";

    cout << "-------------------------\n";
    cout << "Fractional solution\n";
    for (int i=0;i<n;i++)
      cout << "x[" << i+1 << "] = "<< x[i].get(GRB_DoubleAttr_X) << endl;
    cout << endl;
    return(model.get(GRB_DoubleAttr_ObjVal));
  }catch (...) {  cout << "Error: Could not obtain a solution."  << endl;}
  return(-1);
}

int integer_packing() 
{
  int n=4;
  vector<GRBVar> x(n);
  GRBEnv env = GRBEnv();
  GRBModel model = GRBModel(env);

  for (int i=0;i<n;i++) x[i]=model.addVar(0.0,GRB_INFINITY,1.0,GRB_INTEGER,"");
  model.update(); // run update to use model inserted variables
  model.addConstr(3*x[0] + 8*x[1] +          2*x[3] >= 950);
  model.addConstr(2*x[0] +            x[2] + 3*x[3] >= 1150);
  model.addConstr(  x[0] +          3*x[2] +   x[3] >= 495);
  model.addConstr(  x[0] +          2*x[2] +   x[3] >= 450);

  model.set(GRB_StringAttr_ModelName, "Packing Example"); 
  model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE); 
  try{
    model.optimize();
    if (model.get(GRB_IntAttr_SolCount) == 0)  // if could not obtain a solution
      throw "Could not obtain a solution.";
    cout << "\n---------------------------------------------------\n";
    cout << "Integer solution\n";
    for (int i=0;i<n;i++)
      cout << "x[" << i+1 << "] = "<< x[i].get(GRB_DoubleAttr_X) << endl;
    cout << endl;
    return(model.get(GRB_DoubleAttr_ObjVal));
  }catch (...) {  cout << "Error: Could not obtain a solution."  << endl;}
  return(-1);
}


int main()
{
  cout << endl << endl <<"Relaxation of the integer linear program in page 124 of the slides in " << endl <<
    "http://www.ic.unicamp.br/~fkm/lectures/proglin.pdf" << endl << endl << endl;

  // The only difference between the fractional and integer model,
  // are the type of the variables. In the first, they are GRB_CONTINUOUS and the last
  // is GRB_INTEGER
  fractional_packing();
  integer_packing();
}
  
