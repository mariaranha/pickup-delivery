//
// Project and Analysis of Algorithms
// Flávio Keidi Miyazawa
//
// ex_viewgrahp.cpp
#include <stdio.h>
#include <lemon/list_graph.h>
#include "mygraphlib.h"
#include <string>
#include "myutils.h"
using namespace lemon;


int main(int argc, char *argv[]) 
{
  Graph g;
  string filename;
  // uncomment one of these lines to change default pdf reader, or insert new one
  //set_pdfreader("open");    // pdf reader for Mac OS X
  //set_pdfreader("xpdf");    // pdf reader for Linux
  //set_pdfreader("evince");  // pdf reader for Linux

  if (argc!=2) {cout<< endl << "Usage: "<< argv[0]<<" <graph_filename>"<<endl << endl <<
      "Example:" << endl <<
      "\t" << argv[0] << " "<< getpath(argv[0])+"../instances/s_50.gr" << endl <<
      "\t" << argv[0] << " "<< getpath(argv[0])+"../instances/t_100.gr" << endl << endl; exit(0);}
  else if (!FileExists(argv[1])) {cout<<"File "<<argv[1]<<" does not exist."<<endl; exit(0);}
  filename = argv[1];

  string type = GetGraphFileType(filename);
  if (type!="graph"){cout<<"Error: Unknown type of graph: "<<type<<endl;exit(1);}

  GraphTable GT(filename,g); // Read the graph (only nodes and edges)
  GT.print();
  return(1);
}

