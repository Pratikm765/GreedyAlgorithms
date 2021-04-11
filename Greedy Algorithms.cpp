//============================================================================
// Name        : Data_Structures.cpp
// Author      : Pratik Mhatre
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <bits/stdc++.h>

using namespace std;

class Edge{

public:
	int v;
	int u;
	int weight;

	Edge(int v,int u,int weight){
		this->v=v;
		this->u=u;
		this->weight=weight;
	}

	 int getU(){
		 return this->u;
	 }

	 int getV(){
		 return this->v;
	 }

	 int getWeight(){
		 return this->weight;
	 }
};

bool operator<(const Edge& p1, const Edge& p2)
{
   return p1.weight > p2.weight;
}

int myComp(const void* a, const void* b)
{
    Edge* a1 = (Edge*)a;
    Edge* b1 = (Edge*)b;
    return a1->weight > b1->weight;
}

class subset{
public:
	int parent;
	int rank;

	subset(){
		parent=-1;
		rank=0;
	}
};

int find(subset subsets[], int i)
{
	if(subsets[i].parent==-1)
		return i;
	return find(subsets,subsets[i].parent);
}

void Union(subset subsets[],int x,int y)
{
    int xroot = find(subsets, x);
    int yroot = find(subsets, y);

    if (subsets[xroot].rank < subsets[yroot].rank)
        subsets[xroot].parent = yroot;
    else if (subsets[xroot].rank > subsets[yroot].rank)
        subsets[yroot].parent = xroot;
    else {
        subsets[yroot].parent = xroot;
        subsets[xroot].rank++;
    }
}

void printSet(subset subsets[],int v){
	for(int i=0;i<v;i++)
		cout<<subsets[i].parent;
	cout<<endl;
}

class Graph{
	int v;
	list<Edge>* adjList;
	vector<Edge> edges;
	int E;

public:
	Graph(int v){
		this->v=v;
		adjList=new list<Edge>[v];
		E=0;
	}

	void addEdge(int v,int u,int weight){
		Edge e1(v,u,weight);
		adjList[v].push_back(e1);

		Edge e2(u,v,weight);
		adjList[u].push_back(e2);

		edges.push_back(e1);
		E++;
	}

	void printadjlist(){
		list<Edge>::iterator it;
		for(int i=0;i<v;i++)
		{
			cout<<"for "<<i<<": ";
			for(it=adjList[i].begin();it!=adjList[i].end();it++)
			{
				cout<<it->v<<"-->"<<it->u<<"@"<<it->weight<<" ";
			}
			cout<<endl;
		}
	}


	void prims(){
		priority_queue<Edge> q;
		q.push(Edge(0,-1,0));
		bool visited[v]={false};

		while(!q.empty()){
			Edge e=q.top();
			q.pop();
			//cout<<"here ";
			if(visited[e.v]==true)
				continue;

			visited[e.v]=true;

			if(e.u!=-1)
				cout<<e.v<<"-->"<<e.u<<" @"<<e.weight<<endl;

			list<Edge>::iterator it;

			for(it=adjList[e.v].begin();it!=adjList[e.v].end();it++)
			{	//cout<<it->u<<" ";
				if(visited[it->u]==false)
				{
					//cout<<"pushed ";
					q.push(Edge(it->u,e.v,it->weight));
				}
			}
		}
	}

	void kruskalsMST(){
		priority_queue<Edge> q;
		for(int i=0;i<E;i++){
			q.push(edges[i]);
		}

		int i=0;
		int j=0;
		vector<Edge> MST;

		subset* st=new subset[v];

		while(i<(v-1) && j<E){
			Edge next_edge = q.top();
			q.pop();
			//cout<<next_edge.u<<" "<<next_edge.v<<" "<<next_edge.weight<<endl;
			j++;
			int x=find(st,next_edge.u);
			int y=find(st,next_edge.v);
			//cout<<x<<"-->"<<y<<endl;

			if(x!=y){
				MST.push_back(next_edge);
				i++;
				Union(st,x,y);
				//cout<<"entered";
			}
			//printSet(st,v);

		}

		for(int i=0;i<MST.size();i++)
			cout<<MST[i].u<<"-->"<<MST[i].v<<endl;

	}
};



void PrimsKruskalWithAdjList(){
	Graph g(9);

    g.addEdge(0, 1, 4);
    g.addEdge(0, 7, 8);
    g.addEdge(1, 2, 8);
    g.addEdge(1, 7, 11);
    g.addEdge(2, 3, 7);
    g.addEdge(2, 8, 2);
    g.addEdge(2, 5, 4);
    g.addEdge(3, 4, 9);
    g.addEdge(3, 5, 14);
    g.addEdge(4, 5, 10);
    g.addEdge(5, 6, 2);
    g.addEdge(6, 7, 1);
    g.addEdge(6, 8, 6);
    g.addEdge(7, 8, 7);

    //g.printadjlist();

    g.prims();

    g.kruskalsMST();
}

int minKey(int key[],bool visited[],int v)
{
	int min=INT_MAX;
	int minIndex;

	for(int i=0;i<v;i++)
	{
		if(visited[i]==false &&  min>key[i])
		{
			min=key[i];
			minIndex=i;
		}
	}
	return minIndex;
}

void PrimsWithAdjMatrix(){

	int V=5;
    int graph[V][V] = { { 0, 2, 0, 6, 0 },
                        { 2, 0, 3, 8, 5 },
                        { 0, 3, 0, 0, 7 },
                        { 6, 8, 0, 0, 9 },
                        { 0, 5, 7, 9, 0 } };

    int parent[V];
    int key[V];
    bool visited[V];

    for (int i = 0; i < V; i++)
         key[i]=INT_MAX,visited[i]=false;

    key[0]=0;
    parent[0]=-1;

    for(int i=0;i<V-1;i++)
    {
    	int u=minKey(key,visited,V);

    	visited[u]=true;
    	for(int v=0;v<V;v++){
    		if(graph[u][v] && visited[v]==false && graph[u][v]<key[v])
    			parent[v]=u,key[v]=graph[u][v];
    	}
    }

    for(int i=1;i<V;i++)
    	cout<<parent[i]<<"<-->"<<i<<" @"<<graph[parent[i]][i]<<endl;

}

void DijsktraWithAdjMatrix(){

	int V=9;
    int graph[V][V] = { { 0, 4, 0, 0, 0, 0, 0, 8, 0 },
            { 4, 0, 8, 0, 0, 0, 0, 11, 0 },
            { 0, 8, 0, 7, 0, 4, 0, 0, 2 },
            { 0, 0, 7, 0, 9, 14, 0, 0, 0 },
            { 0, 0, 0, 9, 0, 10, 0, 0, 0 },
            { 0, 0, 4, 14, 10, 0, 2, 0, 0 },
            { 0, 0, 0, 0, 0, 2, 0, 1, 6 },
            { 8, 11, 0, 0, 0, 0, 1, 0, 7 },
            { 0, 0, 2, 0, 0, 0, 6, 7, 0 } };

    int key[V];
    bool visited[V];

    for (int i = 0; i < V; i++)
         key[i]=INT_MAX,visited[i]=false;

    key[0]=0;

    for(int i=0;i<V-1;i++)
    {
    	int u=minKey(key,visited,V);
    	visited[u]=true;
    	for(int v=0;v<V;v++){
    		if(graph[u][v] && visited[v]==false && key[u]!=INT_MAX && key[u]+graph[u][v]<key[v])
    			key[v]=key[u]+graph[u][v];
    	}
    }

    for(int i=0;i<V;i++)
    	cout<<i<<"<-->"<<key[i]<<endl;

}
void BellmanFordUtil(int g[][3],int V, int E,int src)
{
	int disc[V];

	for(int i=0;i<V;i++)
	{
		disc[i]=INT_MAX;
	}
	disc[src]=0;

	for(int i=0;i<V-1;i++)
	{
		for(int j=0;j<E;j++)
		{
			if(disc[g[j][0]]!=INT_MAX &&  disc[g[j][0]]+g[j][2] < disc[g[j][1]])
				disc[g[j][1]]= disc[g[j][0]]+g[j][2];
		}
	}

	for(int i=0;i<E;i++)
	{
		if(disc[g[i][0]]!=INT_MAX && disc[g[i][0]]+g[i][2]<disc[g[i][1]])
		{
			cout<<"Negative Cycle"<<endl;
		}
	}

	for(int i=0;i<V;i++)
	{
		cout<<i<<" "<<disc[i]<<endl;
	}

}

void BellmanFord(){
	int V = 5; // Number of vertices in graph
	int E = 8; // Number of edges in graph


	int graph[][3] = { { 0, 1, -1 }, { 0, 2, 4 },
	                       { 1, 2, 3 }, { 1, 3, 2 },
	                       { 1, 4, 2 }, { 3, 2, 5 },
	                       { 3, 1, 1 }, { 4, 3, -3 } };

	BellmanFordUtil(graph, V, E, 0);
}


void floydWarshall(){
	int V=4;
	int MAX=99999;
    int graph[V][V] = { {0, 5, MAX, 10},
                        {MAX, 0, 3, MAX},
                        {MAX, MAX, 0, 1},
                        {MAX, MAX, MAX, 0}
                    };
    int dist[V][V];
    for(int i=0;i<V;i++)
    	for(int j=0;j<V;j++)
    		dist[i][j]=graph[i][j];

    for(int i=0;i<V;i++)
    {
    	for(int j=0;j<V;j++)
    	{
    		if(dist[i][j]==MAX)
    			cout<<"INF ";
    		else
    			cout<<dist[i][j]<<" ";
    	}
    	cout<<endl;
    }

    for(int i=0;i<V;i++)
    {
    	for(int j=0;j<V;j++)
    	{
    		for(int k=0;k<V;k++)
    		{
    			cout<<j<<i<<" "<<i<<k<<" "<<j<<k<<endl;
    			//cout<<dist[j][k]<< " "<<dist[j][i]<<" "<<dist[i][k]<<endl;
    			if(dist[j][i]+dist[i][k]<dist[j][k])
    				dist[j][k]=dist[j][i]+dist[i][k];
    			//cout<<dist[j][k]<<endl;
    		}
    	}
    }

    for(int i=0;i<V;i++)
    {
    	for(int j=0;j<V;j++)
    	{
    		if(dist[i][j]==MAX)
    			cout<<"INF ";
    		else
    			cout<<dist[i][j]<<" ";
    	}
    	cout<<endl;
    }
}


class GraphDir
{
    int V;
    list<int> *adj;
    void fillOrder(int v, bool visited[], stack<int> &Stack){
    	visited[v]=true;
    	list<int>::iterator it;

    	for(it=adj[v].begin();it!=adj[v].end();it++)
    	{
    		if(!visited[*it])
    			fillOrder(*it,visited,Stack);
    	}
    	Stack.push(v);
    }
    void DFSUtil(int v, bool visited[]){
    	visited[v]=true;
    	list<int>::iterator it;
    	cout<<v<<" ";
    	for(it=adj[v].begin();it!=adj[v].end();it++)
    	{
    		if(!visited[*it])
    			DFSUtil(*it,visited);
    	}
    }
public:
    GraphDir(int V){
    	this->V=V;
    	adj=new list<int>[V];
    }
    void addEdge(int v, int w){
    	adj[v].push_back(w);
    }

    void addEdgeUndir(int v, int w)
    {
        adj[v].push_back(w);
        adj[w].push_back(v);  // Note: the graph is undirected
    }

    void printSCCs(){

    	stack<int> st;
    	bool visited[V];

    	for(int i=0;i<V;i++)
    	{
    		visited[i]=false;
    	}

    	for(int i=0;i<V;i++)
    	{
    		if(!visited[i])
    			fillOrder(i,visited,st);
    	}

    	GraphDir gt= getTranspose();

    	for(int i=0;i<V;i++)
    	{
    		visited[i]=false;
    	}

    	while(!st.empty()){
    		int s= st.top();
    		st.pop();

    		if(!visited[s])
    			gt.DFSUtil(s,visited);
    		cout<<endl;
    	}
    }

    GraphDir getTranspose()
    {
    	GraphDir g(V);

    	list<int>::iterator it;
    	for(int i=0;i<V;i++){
    		for(it=adj[i].begin();it!=adj[i].end();it++)
    			g.addEdge(*it,i);
    	}

    	return g;
    }

    void tarjansSCCUtil(int u,int disc[],int low[],bool visited[],stack<int> &st){
    	static int time=0;

    	disc[u]=low[u]=++time;
    	st.push(u);
    	visited[u]=true;

    	list<int>::iterator it;
    	for(it=adj[u].begin();it!=adj[u].end();it++){
    		//if it is not visited
    		if(disc[*it]==-1)
    		{
    			tarjansSCCUtil(*it,disc,low,visited,st);
    			//normal edge update low values based on child nodes
    			low[u]=min(low[*it],low[u]);
    		}
    		//If it is back edge not a cross edge
    		else if(visited[*it]==true)
    			low[u]=min(disc[*it],low[u]);
    	}

    	int w=0;
    	//we got head node
    	if(disc[u]==low[u])
    	{
    		//pop till the u node
    		while(st.top()!=u){
        		w=st.top();
        		st.pop();
        		visited[w]=false;
        		cout<<w<<" ";
    		}

    		w=st.top();
    		st.pop();
    		visited[w]=false;
    		cout<<w<<endl;
    	}
    }

    void tarjansSCC(){

    	int disc[V];
    	int low[V];
    	bool visited[V];
    	stack<int> st;

    	for(int i=0;i<V;i++){
    		disc[i]=-1;
    		low[i]=-1;
    		visited[i]=false;
    	}

    	for(int i=0;i<V;i++){
    		if(disc[i]==-1)
    			tarjansSCCUtil(i,disc,low,visited,st);
    	}
    }

    //for undirected
    void bridgeUtil(int u,bool visited[],int disc[],int low[],int parent[]){
    	static int time=0;

    	disc[u]=low[u]=++time;
    	visited[u]=true;

    	list<int>::iterator it;

    	for(it=adj[u].begin();it!=adj[u].end();it++){
    		//Not visited
    		if(!visited[*it]){
    			parent[*it]=u;

    			bridgeUtil(*it,visited,disc,low,parent);
    			low[u]=min(low[u],low[*it]);

    			//if no back edge
    			if(low[*it]>disc[u])
    				cout<<u<<" "<<*it<<endl;
    		}
    		//Not same edge child to parent
    		else if(parent[u]!=*it)
    			low[u]=min(low[u],disc[*it]);
    	}
    }

    void bridgeTarjan(){

    	int disc[V];
    	int low[V];
    	bool visited[V];
    	int parent[V];

    	for(int i=0;i<V;i++){
    		disc[i]=-1;
    		low[i]=-1;
    		parent[i]=-1;
    		visited[i]=false;
    	}

    	for(int i=0;i<V;i++){
    		if(!visited[i])
    			bridgeUtil(i,visited,disc,low,parent);
    	}
    }

    //for undirected
    void ArticulationPointUtil(int u,bool visited[],int disc[],int low[],int parent[],bool ap[]){
    	static int time=0;
    	int children=0;

    	disc[u]=low[u]=++time;
    	visited[u]=true;

    	list<int>::iterator it;

    	for(it=adj[u].begin();it!=adj[u].end();it++){
    		if(!visited[*it]){
    			parent[*it]=u;
    			children++;
    			ArticulationPointUtil(*it,visited,disc,low,parent,ap);
    			low[u]=min(low[u],low[*it]);

    			if(parent[u]==-1 && children>1)
    				ap[u]=true;
    			if(parent[u]!=-1 && low[*it]>=disc[u])
    				ap[u]=true;
    		}
    		else if(parent[u]!=*it)
    			low[u]=min(low[u],disc[*it]);

    	}
    }

    void articulationPoint(){
    	int disc[V];
    	int low[V];
    	bool visited[V];
    	int parent[V];
    	bool ap[V];

    	for(int i=0;i<V;i++){
    		parent[i]=-1;
    		visited[i]=false;
    		ap[i]=false;
    	}

    	for(int i=0;i<V;i++){
    		if(!visited[i])
    			ArticulationPointUtil(i,visited,disc,low,parent,ap);
    	}

    	cout<<"Articulation Points are: ";
    	for(int i=0;i<V;i++){
    		if(ap[i]==true)
    			cout<<i<<" ";
    	}
    	cout<<endl;
    }
};

void kosaRajuSCC(){

    GraphDir g(5);
    g.addEdge(1, 0);
    g.addEdge(0, 2);
    g.addEdge(2, 1);
    g.addEdge(0, 3);
    g.addEdge(3, 4);

    cout << "Following are strongly connected components in "
            "given graph \n";
    g.printSCCs();
}

void tarjanSCC(){

    GraphDir g4(11);
    g4.addEdge(0,1);g4.addEdge(0,3);
    g4.addEdge(1,2);g4.addEdge(1,4);
    g4.addEdge(2,0);g4.addEdge(2,6);
    g4.addEdge(3,2);
    g4.addEdge(4,5);g4.addEdge(4,6);
    g4.addEdge(5,6);g4.addEdge(5,7);g4.addEdge(5,8);g4.addEdge(5,9);
    g4.addEdge(6,4);
    g4.addEdge(7,9);
    g4.addEdge(8,9);
    g4.addEdge(9,8);
    g4.tarjansSCC();
}

void bridgeArticulationTarjans(){
	//actually it is unweighted graph
    GraphDir g1(5);
    g1.addEdgeUndir(1, 0);
    g1.addEdgeUndir(0, 2);
    g1.addEdgeUndir(2, 1);
    g1.addEdgeUndir(0, 3);
    g1.addEdgeUndir(3, 4);
    g1.bridgeTarjan();
    g1.articulationPoint();
}



int main() {

	//Minimum Spanning Tree
	PrimsKruskalWithAdjList();
	//PrimsWithAdjMatrix();

	//Shortest Distance
	//DijsktraWithAdjMatrix();
	//BellmanFord();
	//floydWarshall();

	//Strongly Connected
	//kosaRajuSCC();
	//tarjanSCC();

	//Using Tarjans Bridge Gap and Articulation point
	//bridgeArticulationTarjans();


	return 0;
}
