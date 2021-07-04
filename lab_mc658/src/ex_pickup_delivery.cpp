// Project and Analysis of Algorithms
// 
// Laboratorio 1
//
// Send corrections/comments to Fl�vio K. Miyazawa
#include <iostream>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <queue>
#include <lemon/list_graph.h>
#include "mygraphlib.h"
#include <string>
#include "myutils.h"
#include <lemon/concepts/digraph.h>
#include <lemon/preflow.h>
using namespace lemon;
using namespace std;

typedef vector<DNode> DNodeVector;

int cutcount = 0;

//Change this value to the number of nodes of the instance before running the algorithm
const int N = 12;

int visited[N];
double minValue = INT_MAX;
double final_res = INT_MAX;
int greedyPath[N];

// Pickup_Delivery_Instance put all relevant information in one class.
class Pickup_Delivery_Instance {
public:
  Pickup_Delivery_Instance(Digraph &graph,
			   DNodeStringMap &vvname,
			   DNodePosMap &posx,
			   DNodePosMap &posy,
			   ArcValueMap &eweight,
			   DNode &sourcenode,
			   DNode &targetnode,
			   int &npairs,
			   DNodeVector& pickup,
			   DNodeVector& delivery);
  Digraph &g;
  DNodeStringMap &vname;
  DNodePosMap &px;
  DNodePosMap &py;
  ArcValueMap &weight;
  int nnodes;
  DNode &source;
  DNode &target;
  int npairs;
  DNodeVector &pickup;
  DNodeVector &delivery;
};

Pickup_Delivery_Instance::Pickup_Delivery_Instance(Digraph &graph,
				 DNodeStringMap &vvname,
				 DNodePosMap &posx,
				 DNodePosMap &posy,
				 ArcValueMap &eweight,
				 DNode &sourcenode,
						   DNode &targetnode,
						   int& vnpairs,
						   DNodeVector& vpickup,
						   DNodeVector& vdelivery):
  g(graph), vname(vvname), px(posx), py(posy), weight(eweight),
  source(sourcenode), target(targetnode),npairs(vnpairs),pickup(vpickup),delivery(vdelivery)
{
  nnodes = countNodes(g);
}

void PrintInstanceInfo(Pickup_Delivery_Instance &P)
{
  cout << endl << endl;
  cout << "Pickup Delivery Graph Informations" << endl;
  cout << "\tTotal de nos: "<< P.nnodes << endl;
  cout << "\tTotal de pares: "<< P.npairs << endl;
  // cout << "\tSource = " << P.vname[P.source] << endl;
  // cout << "\tTarget = " << P.vname[P.target] << endl;
  // for (int i=0;i<P.npairs;i++) {
  //   cout << "\tPair pickup-->delivery: "
	//  << P.vname[P.pickup[i]]
	//  << " --> "
	//  << P.vname[P.delivery[i]] << endl;
  // }

  // cout << "Impressao do grafo no formato (no_origem, no_destino, peso_arco)"<< endl;
  // for(DNodeIt v(P.g);v!=INVALID;++v) {
  //   cout << P.vname[v] << ": ";
  //   for (OutArcIt a(P.g,v);a!=INVALID;++a) {
  //     cout << "("<<
	// P.vname[P.g.source(a)] << "," <<
	// P.vname[P.g.target(a)] << "," <<
	// P.weight[a] << ")  "; }
  //   cout<<endl;
  // }

  // cout<<endl<<endl;
}

void PrintSolution(Pickup_Delivery_Instance &P,DNodeVector &Sol,string msg)
{
  // Imprime a solucao no terminal.
  cout<<msg<<endl<< "\t";
  cout << P.vname[Sol[0]];
  for (int i=1;i<P.nnodes;i++)
    cout << "-->" << P.vname[Sol[i]];
  cout<<endl;

}

bool ReadPickupDeliveryDigraph(string filename,
			       Digraph &g,
			       DNodeStringMap& vname,
			       DNodePosMap& posx,
			       DNodePosMap& posy,
			       ArcValueMap& weight,
			       DNode& source,
			       DNode& target,
			       int &npairs,
			       DNodeVector& pickup,
			       DNodeVector& delivery)
{
  ReadDigraph(filename,g,vname,posx,posy,weight);
  int n=countNodes(g);
  DNode DN[n];
  if ((n<4) || (n%2)) {
    cout << "Numero de vertices "<<n<<" no grafo nao eh par ou eh menor que 4." << endl;
    return(0);
  }
  npairs = (n-2)/2;
  pickup.resize(npairs);
  delivery.resize(npairs);
  int i=0;
  for(DNodeIt v(g);v!=INVALID;++v) { DN[i] = v; i++; }
  
  source = DN[0];
  target = DN[1];
  for (int i=0;i<npairs;i++) {
    pickup[i] = DN[2*i+2];
    delivery[i] = DN[2*i+3];
  }
  return(1);
}

double sqr(double a) {  return(a*a); }
double dist(Pickup_Delivery_Instance &P,DNode u,DNode v)
{  return(sqrt(sqr(P.px[u]-P.py[v])+sqr(P.px[v]-P.py[v]))); }

// Heuristica apenas para testar a visualizacao das solucoes.
bool HeuristicaConstrutivaBoba(Pickup_Delivery_Instance &P,int time_limit,
			       double &LB,double &UB,DNodeVector &Sol)
{
  cout << "Execucao da Heuristica Boba" << endl;
  cout << "\tEsta rotina deveria respeitar o tempo de no maximo "
       << time_limit << " segundos" << endl;
  cout<<"\tProvavelmente a solucao eh inviavel, pois mistura as coletas e entregas."<< endl;
  if (UB==MY_INF) { // Faz alguma coisa so' se ainda nao tem solucao
    Sol.resize(P.nnodes);
    Sol[0] = P.source; // insere o source
    for (int i=0;i<P.npairs;i++) Sol[i+1] = P.pickup[i]; // insere os pickup
    for (int i=0;i<P.npairs;i++) Sol[P.npairs+i+1] = P.delivery[i]; // insere os delivery
    Sol[2*P.npairs+1] = P.target; // insere o target.

    // Ordena os vertices, exceto pelo source e target, do mais baixo para mais alto
    int primeiro=1, ultimo=2*P.npairs;
    for (int i=primeiro;i<ultimo;i++) 
      for (int j=i+1;j<=ultimo;j++) 
	if (P.py[Sol[i]]>P.py[Sol[j]]){DNode aux;aux=Sol[i];Sol[i]=Sol[j];Sol[j]=aux;}

    // Atualiza o UB (Upper Bound) que eh o valor da solucao
    UB = 0.0;
    for (int i=1;i<P.nnodes;i++)
      for (OutArcIt a(P.g,Sol[i-1]);a!=INVALID;++a)
	if(P.g.target(a)==Sol[i]){UB += P.weight[a];break;}
  }
  cout<<endl;
  return(1);
}

bool ViewPickupDeliverySolution(Pickup_Delivery_Instance &P,double &LB,double &UB,DNodeVector &Sol,string msg)
{
  DigraphAttributes GA(P.g,P.vname,P.px,P.py);
  GA.SetDefaultDNodeAttrib("color=LightGray style=filled width=0.2 height=0.2 fixedsize=true");
  for (ArcIt a(P.g); a!=INVALID; ++a) GA.SetColor(a,"Invis");
  GA.SetColor(P.source,"Red"); // source and target are painted in White
  GA.SetColor(P.target,"Red");
  GA.SetAttrib(P.source,"shape=box");
  GA.SetAttrib(P.target,"shape=box");
  
  if (P.npairs <= 16){ // se tiver poucos pares, dah para pintar os pares de mesma cor.
    for (int i=0;i<P.npairs;i++){ // pinta 
      GA.SetColor(P.pickup[i],ith_VisualDistinctColorName(i));
      GA.SetColor(P.delivery[i],ith_VisualDistinctColorName(i));
    }}
  for (int i=1;i<P.nnodes;i++) {
    // pinta o arco Sol[i-1] -->  Sol[i]
    for (OutArcIt a(P.g,Sol[i-1]);a!=INVALID;++a)
      if(P.g.target(a)==Sol[i]){
	GA.SetColor(a,"Red");
	break;
      }
  }
  GA.SetLabel("Path from node "+P.vname[P.source]+" to node "+P.vname[P.target]+
	      " of value "+DoubleToString(UB)+". LB = "+DoubleToString(LB)+ ". "+msg);
  GA.View();
  return(1);
}

void print2DArray(double **arr, int N){
  for (int i = 0; i < N; i++){
    for (int j = 0; j < N; j++){
      cout << arr[i][j] << " ";
    }
    cout << endl;
  }
}

void Lab2(Pickup_Delivery_Instance &P, double **adj, int pickup[], int delivery[], int src, int dest){
  int curr_path[P.nnodes];

  //Build a initial path with all pickups nodes followed by all delivery nodes 
  // curr_path[0] = src;
  // int aux = 1;
  
  // for(int i=0; i<P.npairs; i++){
  //   curr_path[aux] = pickup[i];
  //   aux += 1;
  // }
  // for(int i=0; i<P.npairs; i++){
  //   curr_path[aux] = delivery[i];
  //   aux += 1;
  // }

  // curr_path[P.nnodes-1] = dest;

  //Build initial path with path generated from greedy heuristic
  for(int k=0;k<P.nnodes;k++){
    curr_path[k] = greedyPath[k];
  }

  int improve = 0;

  int new_path[P.nnodes];

  while(improve < 1000){
    double best_cost = 0;
    for(int i=0;i<P.nnodes-1;i++){
      best_cost += adj[curr_path[i]][curr_path[i+1]];
    }

    for(int i=1; i<P.nnodes-2; i++){
      for(int j=i+1; j<P.nnodes-1; j++){
        //copy curr path to new_path
        for(int k=0; k<P.nnodes; k++){
          new_path[k] = curr_path[k];
        }

        //Swap two nodes
        int aux = new_path[i];
        new_path[i] = new_path[j];
        new_path[j] = aux;

        bool isInPath = false;
        bool isDelivery = false;
        bool isPickup = false;
        bool isDeliveryInPath = false;

        for(int n=0; n<P.npairs;n++){
          //Check if vertice i is delivery
          if(new_path[i] == delivery[n]){
            isDelivery = true;
            //Check if correspondent pickup is in path before i
            for(int m=0;m<i;m++){
              if(new_path[m] == pickup[n]){
                isInPath = true;
              }
            }
          }
        }

        for(int n=0; n<P.npairs;n++){
          //Check if vertice i is pickup
          if(new_path[i] == pickup[n]){
            isPickup = true;
            //Check if correspondent delivery is in path before i
            for(int m=0;m<i;m++){
              if(new_path[m] == delivery[n]){
                isDeliveryInPath = true;
              }
            }
          }
        }

        bool isInPath2 = false;
        bool isDelivery2 = false;
        bool isPickup2 = false;
        bool isDeliveryInPath2 = false;

        for(int n=0; n<P.npairs;n++){
          //Check if vertice j is delivery
          if(new_path[j] == delivery[n]){
            isDelivery2 = true;
            //Check if correspondent pickup is in path before j
            for(int m=0;m<j;m++){
              if(new_path[m] == pickup[n]){
                isInPath2 = true;
              }
            }
          }
        }

        for(int n=0; n<P.npairs;n++){
          //Check if vertice i is pickup
          if(new_path[j] == pickup[n]){
            isPickup2 = true;
            //Check if correspondent delivery is in path before i
            for(int m=0;m<j;m++){
              if(new_path[m] == delivery[n]){
                isDeliveryInPath2 = true;
              }
            }
          }
        }

        if((isDelivery == false || isInPath == true) && (isPickup == false || isDeliveryInPath == false) &&
          (isDelivery2 == false || isInPath2 == true) && (isPickup2 == false || isDeliveryInPath2 == false)){
          //calculates cost of new path
          double new_cost = 0;
          for(int l=0;l<P.nnodes-1;l++){
            new_cost += adj[new_path[l]][new_path[l+1]];
          }
          if(new_cost<best_cost){
            improve = 0;
            for(int k=0; k<P.nnodes; k++){
              curr_path[k] = new_path[k];
            }
            best_cost = new_cost;
          }
        }
      }
    }
    improve++;
  }

    //Print initial path
  cout<<"2-Opt Path is: ";
  for(int i=0; i<P.nnodes; i++){
    cout<<curr_path[i]<<" ";
  }
  cout<<endl;

  double final_cost = 0;
  for(int i=0;i<P.nnodes-1;i++){
    final_cost += adj[curr_path[i]][curr_path[i+1]];
  }

  cout<<"2-Opt Final cost is: "<<final_cost<<endl;
}


// Function to copy temporary solution to
// the final solution
// add final node 
void copyToFinal(int curr_path[], int final_path[], int nNodes, int dest)
{
	for (int i=0; i<nNodes; i++)
		final_path[i] = curr_path[i];
}


//Function that finds minimum cost path with pickup-delivey rule
void findMinBound(double **adj, int nNodes, double curr_weight,
			int level, int curr_path[], int src, int dest, int pickup[], int delivery[], int nPairs)
{
  if (level==nNodes-1){
    curr_weight += adj[curr_path[level-1]][dest];
    minValue = curr_weight;
    curr_path[nNodes-1] = dest;

    for(int i=0;i<nNodes;i++){
      greedyPath[i] = curr_path[i];
    }
    
    //Print final path
    cout<<"Greedy Heuristic: "<<endl;
    cout<<"Final path is: ";
    for(int i=0; i<nNodes; i++){
      cout<<curr_path[i]<<" ";
    }
    cout<<endl;
    cout<<"Total cost is: "<<curr_weight<<endl;
    return;
  }

  bool isValid = false;
  bool minVisited[nNodes];
  int index;

  memset(minVisited, false, sizeof(minVisited));

  //Find a valid node to put in path
  while(isValid == false){

    //Finds min node candidate
    index = -1;
    double minCost = INT_MAX;

    for(int i=0; i<nNodes;i++){
      if(visited[i] == true || minVisited[i] == true || adj[level-1][i] == 0){
        continue;
      }
      if(adj[level-1][i] < minCost){
        index = i;
        minCost = adj[level-1][i];
      }
    }
    minVisited[index] = true;

    int indexDelivery = -1;

    //Check if vertex belongs to delivery
    for(int j=0; j < nPairs; j++){
      if(index == delivery[j]){
          indexDelivery = j;
        }
    }

    bool isInputInPath = false;

    //Check if correspondent input vertice is already in curr_path
    if(indexDelivery != -1){
      for(int j=0; j < level; j++){
        if(curr_path[j] == pickup[indexDelivery]){
          isInputInPath = true;
        }
      }
    }

    if(isInputInPath == true || indexDelivery == -1){
      isValid = true;
    }
  }

  curr_weight += adj[curr_path[level-1]][index];
  curr_path[level] = index;
  visited[index] = true;

  findMinBound(adj, nNodes, curr_weight, level+1,
  curr_path, index, dest, pickup, delivery, nPairs);
}



//Branch and Bound function that finds minimum cost path with pickup-delivey rule
void BranchAndBound(double **adj, int nNodes, double curr_bound, double curr_weight,
			int level, int curr_path[], int final_path[], int src, int dest, int pickup[], int delivery[], int nPairs, double lower_bound)
{
  //Base case when we have already covered all levels of
	//the tree
	if (level==nNodes-1){	

		//Add target node
		curr_path[nNodes-1] = dest;

		//Add final node cost
		float curr_res = curr_weight +
				adj[curr_path[level-1]][dest];

		//Update final result is current one is better
		if (curr_res < final_res && curr_res < lower_bound)
		{
			copyToFinal(curr_path, final_path, nNodes, dest);
			final_res = curr_res;
		}
		return;
	}

	//Build search space tree for other levels
	for (int i=0; i<nNodes; i++)
	{
    int indexDelivery = -1;

    //Check if vertex belongs to delivery
    for(int j=0; j < nPairs; j++){
      if(i == delivery[j]){
          indexDelivery = j;
        }
    }

    bool isInputInPath = false;

    //Check if correspondent input vertice is already in curr_path
    if(indexDelivery != -1){
      for(int j=0; j < level; j++){
        if(curr_path[j] == pickup[indexDelivery]){
          isInputInPath = true;
        }
      }
    }      

    //If it has not been already visited and is valid according to pickup-delivery rule,
    //then open node
		if (adj[curr_path[level-1]][i] != 0 &&
			visited[i] == false && (indexDelivery == -1 || isInputInPath == true))
		{
			curr_weight += adj[curr_path[level-1]][i];

      double naive_bound = nNodes * curr_weight / level;

			//Explore the node futher
			if (curr_weight < final_res && naive_bound < lower_bound)
			{
				curr_path[level] = i;
				visited[i] = true;
				visited[dest] = true;
				
				BranchAndBound(adj, nNodes, curr_bound, curr_weight, level+1,
					curr_path, final_path, src, dest, pickup, delivery, nPairs, lower_bound);
			}
			//Reset changes to curr_weight and curr_bound
			curr_weight -= adj[curr_path[level-1]][i];

			//Mark only nodes in current path as visited
			memset(visited, false, sizeof(visited));
			for (int j=0; j<=level-1; j++){
				visited[curr_path[j]] = true;
      }
		}
	}
}

bool Lab1(Pickup_Delivery_Instance &P,int time_limit,double &LB,double &UB,DNodeVector &Sol)
{
  /*Transform graph into a 2d array -------------*/
  double **Dist = new double*[P.nnodes];

  for (int i = 0; i < P.nnodes; i++){
    Dist[i] = new double[P.nnodes];
  }

  int verticeinicio,verticefim;
  // Vamos mapear vertice (um numero de 0 a NumeroDeNos-1) para No (no' no Lemon) e
  // o mapeamento contrario.
  //DNode MapeiaNoVertice[P.nnodes]; // Mapeia int ==> DNode
  DNode MapeiaVerticeNo[P.nnodes]; // Mapeia DNode ==> int
  DNodeIntMap MapeiaNoVertice(P.g);
  // double Dist[P.nnodes][P.nnodes];
  int pickup[P.npairs],delivery[P.npairs],Solucao[P.nnodes];
  verticeinicio = 0;
  verticefim = 2*P.npairs+1;
  MapeiaVerticeNo[verticeinicio] = P.source;
  MapeiaNoVertice[P.source] = verticeinicio;  
  MapeiaVerticeNo[verticefim] = P.target;
  MapeiaNoVertice[P.target] = verticefim;  
  for (int i=0;i<P.npairs;i++) {
    MapeiaVerticeNo[i+1] = P.pickup[i]; // insere os pickup
    MapeiaNoVertice[P.pickup[i]] = i+1;
    pickup[i] = i+1;
  }
  for (int i=0;i<P.npairs;i++) {
    MapeiaVerticeNo[P.npairs+i+1] = P.delivery[i]; // insere os delivery
    MapeiaNoVertice[P.delivery[i]] = P.npairs+i+1;
    delivery[i] = P.npairs+i+1;
  }
  // Inicializa a matriz com Dist infinito em todos os arcos (i-->j)
  for (int i=0;i<P.nnodes;i++) 
    for (int j=0;j<P.nnodes;j++)
      Dist[i][j] = MY_INF;
  // Distancia de i para i eh zero.
  for (int i=0;i<P.nnodes;i++)  Dist[i][i] = 0;
  // Atribui o peso dos arcos na matriz completa vindo de arcos do grafo.
  for (ArcIt a(P.g); a!=INVALID; ++a) {
    DNode n_i,n_j;
    int v_i,v_j;
    n_i = P.g.source(a);   // Pega os no's do arco (i-->j)
    n_j = P.g.target(a);
    v_i = MapeiaNoVertice[n_i];
    v_j = MapeiaNoVertice[n_j];
    Dist[v_i][v_j] = P.weight[a];
  }

  // Uncomment these lines to print distance matrix
  // cout<<"Matriz de Distâncias"<<endl;
  // print2DArray(Dist, P.nnodes);

  int vPickup[P.npairs];
  int vDelivery[P.npairs];
  int src = stoi(P.vname[P.source])-1;
  int dest = stoi(P.vname[P.target])-1;

  for(int i=0; i < P.npairs; i++){
    vPickup[i] = stoi(P.vname[P.pickup[i]])-1;
    vDelivery[i] = stoi(P.vname[P.delivery[i]])-1;
  }

  
  cout<<"\tSource = "<<src<<endl;
  cout<<"\tTarget = "<<dest<<endl;
  cout<<"\tPares Pickup->Delivery"<<endl;
  for(int i=0; i<P.npairs; i++){
    cout<<"\t"<<vPickup[i]<<" --> "<<vDelivery[i]<<endl;
  }
  cout<<endl<<endl;

//*---------------------------------------------------*/

  clock_t timeBegin = clock();

	int curr_path[P.nnodes];
  int final_path[P.nnodes];
	double curr_bound = 0;

  //Set curr_path and final_path as empty and set all nodes as not visited 
	memset(curr_path, -1, sizeof(curr_path));
	memset(final_path, -1, sizeof(final_path));
	memset(visited, 0, sizeof(visited));

	//Insert source vertice in curr_path and mark 
  //and mark both sorce and target as visited
	visited[src] = true;
  visited[dest] = true;
	curr_path[0] = src;

  memset(greedyPath, -1, sizeof(greedyPath));

  //Execute solution with greedy approach that looks for least cost edges
  clock_t greedyHeuristicTimer= clock();
  findMinBound(Dist, P.nnodes, 0.0, 1, curr_path, src, dest, vPickup, vDelivery, P.npairs);
  printf("Time taken: %.5fs\n", (double)(clock() - greedyHeuristicTimer)/CLOCKS_PER_SEC);

  //2-opt Heuristic
  clock_t optHeuristicTimer = clock();
  Lab2(P, Dist, vPickup, vDelivery, src, dest);
  printf("Time taken: %.5fs\n", (double)(clock() - optHeuristicTimer)/CLOCKS_PER_SEC);

  double lower_bound = minValue;

  memset(curr_path, -1, sizeof(curr_path));
	memset(visited, 0, sizeof(visited));
  visited[src] = true;
  visited[dest] = true;
	curr_path[0] = src;

  BranchAndBound(Dist, P.nnodes, curr_bound, 0, 1, curr_path, final_path, 
              src, dest, vPickup, vDelivery, P.npairs, lower_bound);

  //Print final cost and path
  cout<<endl<<endl;
  cout<<"Branch and Bound"<<endl;
  cout<<"Menor custo: "<<final_res<<endl;
	cout<<"Caminho percorrido: "<<endl;
	for (int i=0; i<P.nnodes-1; i++)
	  cout<<final_path[i]<<" -> ";
  cout<<final_path[P.nnodes-1]<<endl;

  printf("Time taken: %.5fs\n", (double)(clock() - timeBegin)/CLOCKS_PER_SEC);
  cout<<endl;

    //Delete 2d array created
  for (int i = 0; i < P.nnodes; i++) //To delete the inner arrays
    delete[] Dist[i];
  delete[] Dist;

  return(1);
}


int main(int argc, char *argv[]) 
{
  int maxtime;
  Digraph g;  // graph declaration
  string digraph_filename, source_node_name, target_node_name;
  DNodeStringMap vname(g);  // name of graph nodes
  DNodePosMap px(g),py(g);  // xy-coodinates for each node
  DNodeColorMap vcolor(g);// color of nodes
  ArcStringMap aname(g);  // name for graph arcs
  ArcColorMap ecolor(g); // color of edges
  ArcValueMap lpvar(g);    // used to obtain the contents of the LP variables
  ArcValueMap weight(g);   // edge weights
  vector <DNode> V;
  DNode sourcenode,targetnode;
  int seed=0;
  srand48(seed);

  // uncomment one of these lines to change default pdf reader, or insert new one
  set_pdfreader("open");    // pdf reader for Mac OS X
  //set_pdfreader("xpdf");    // pdf reader for Linux
  //set_pdfreader("evince");  // pdf reader for Linux
  //set_pdfreader("open -a Skim.app");
  // double cutoff;   // used to prune non promissing branches (of the B&B tree)
  if (argc!=3) {cout<<endl<<
      "Laboratorio de MC658: Rota com coleta e entrega de peso minimmo," << endl <<
      "the st-shortest path problem." << endl << endl <<
      "Usage: "<< argv[0]<<"  <pickup_delivery_digraph_filename> <maximum_time_sec>"<< endl << endl;
    cout << "Example:" << endl <<
      "\t" << argv[0] << " "<< getpath(argv[0])+"../instances/pickup_delivery_5.dig 10" << endl << endl <<
      "\t" << argv[0] << " "<< getpath(argv[0])+"../instances/pickup_delivery_10.dig 100" << endl << endl;
    exit(0);}

  digraph_filename = argv[1];
  maxtime = atoi(argv[2]);
  DNodeVector pickup,delivery;
  DNode source,target;
  int npairs;
  
  if (!ReadPickupDeliveryDigraph(digraph_filename,g,vname,px,py,weight,
				 source,target,npairs,pickup,delivery)){
    cout << "Erro na leitura do grafo de entrada." << endl;}
    
  Pickup_Delivery_Instance P(g,vname,px,py,weight,source,target,npairs,pickup,delivery);
  PrintInstanceInfo(P);
  
  double LB = 0, UB = MY_INF; // considere MY_INF como infinito.
  DNodeVector Solucao;
  
  bool melhorou = Lab1(P,maxtime,LB,UB,Solucao);

  if (melhorou) {
    ViewPickupDeliverySolution(P,LB,UB,Solucao,"Solucao do Lab.");
    PrintSolution(P,Solucao,"Solucao do Lab1.");
  }
  return 0;
}

