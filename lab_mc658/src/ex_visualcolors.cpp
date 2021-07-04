#include <assert.h>
#include <float.h>
#include <math.h>
#include <set>
#include <lemon/list_graph.h>
#include <lemon/unionfind.h>
#include <lemon/gomory_hu.h>
#include <lemon/adaptors.h>
#include <lemon/connectivity.h>
#include "mygraphlib.h"
#include "myutils.h"


int main(int argc, char *argv[]) 
{
  Graph g;
  EdgeValueMap weight(g);
  NodeStringMap vname(g);
  EdgeStringMap ename(g);
  NodeColorMap vcolor(g);// color of nodes
  EdgeColorMap ecolor(g); // color of edges
  NodePosMap   posx(g),posy(g);
  string filename,graphtitle;


  // uncomment one of these lines to change default pdf reader, or insert new one
  set_pdfreader("open");    // pdf reader for Mac OS X
  //set_pdfreader("xpdf");    // pdf reader for Linux
  //set_pdfreader("evince");  // pdf reader for Linux

  if (argc!=2) {cout<< endl << "Usage: "<< argv[0]<<" <graph_filename>"<<endl << endl <<
      "Example:" << endl <<
      "\t" << argv[0] << " "<< getpath(argv[0])+"../instances/s_50.gr" << endl <<
      "\t" << argv[0] << " "<< getpath(argv[0])+"../instances/t_100.gr" << endl << endl; exit(0);}
  
  else if (!FileExists(argv[1])) {cout<<"File "<<argv[1]<<" does not exist."<<endl; exit(0);}
  filename = argv[1];

  // Read the graph
  if (!ReadGraph(filename,g,vname,posx,posy,weight)) 
    {cout<<"Error reading graph file "<<argv[1]<<"."<<endl;exit(0);}

  GraphAttributes G(g,vname,posx,posy);
  G.SetGraphAttrib("label=Sizes_and_colors");
  G.SetDefaultNodeAttrib("shape=ellipse style=filled color=Gray");
  G.SetDefaultEdgeAttrib("color=black");

  int c=FirstVisualDistinctColor();
  for (NodeIt v(g); v!=INVALID; ++v) {
    G.SetColor(v,ColorRGB(c));
    c = NextVisualDistinctColor(c);
    if (drand48() <1.0/3) G.SetFontSize(v,30);// about 1/3 of the nodes have large font
  }
  for (EdgeIt e(g); e!=INVALID; ++e) {
    G.SetLabel(e,weight[e]);G.SetColor(e,"Blue");
    if (drand48() <1.0/3)   G.SetStyle(e,"dashed");// ~1/3 of the edges are dashed
    if (drand48() <1.0/3)   G.SetFontSize(e,30);}// ~1/3 of the edges have large font
  G.SetImageFactor(1.2); // Increase the image by factor of 1.2
  G.View();
}
  
