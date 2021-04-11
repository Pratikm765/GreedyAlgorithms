//============================================================================
// Name        : Data_Structures.cpp
// Author      : Pratik Mhatre
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <bits/stdc++.h>
#define row 5
#define col 5
using namespace std;

class Graph{
	int v;
	list<int> * adjList;
public:
	Graph(int v){
		this->v=v;
		adjList= new list<int>[v];
	}
	void addEdge(int v,int w){
		adjList[v].push_back(w);
	}
	void addEdgeUndir(int v,int w){
		adjList[v].push_back(w);
		adjList[w].push_back(v);
	}
	void printadjlist(){
		list<int>::iterator it;
		for(int i=0;i<v;i++)
		{
			cout<<"for "<<i<<": ";
			for(it=adjList[i].begin();it!=adjList[i].end();it++)
			{
				cout<<*it<<" ";
			}
			cout<<endl;
		}
	}

	void bfs(int s){

		bool visited[v];
		for(int i=0;i<v;i++)
			visited[i]=false;

		queue<int> q;
		q.push(s);
		visited[s]=true;

		list<int>::iterator it;

		while(!q.empty())
		{
			s=q.front();
			cout<<s<<" ";
			q.pop();

			for(it=adjList[s].begin();it!=adjList[s].end();it++)
			{
				if(!visited[*it])
				{
					visited[*it]=true;
					q.push(*it);
				}
			}
		}

	}

	void DFSUtil(int v,bool visited[]){
		visited[v]=true;
		cout<<v<<" ";

		list<int>::iterator it;

		for(it=adjList[v].begin();it!=adjList[v].end();it++)
		{
			//cout<<v<<" "<<"it: "<<*it<<" "<<visited[*it]<<endl;
			if(!visited[*it])
			{
				DFSUtil(*it,visited);
			}
		}
	}

	void dfs(int s){
		bool *visited=new bool[v];
		for(int i=0;i<v;i++)
			visited[i]=false;

		DFSUtil(s,visited);
	}

	bool checkCycleUtil(int s,bool visited[], int parent){
		visited[s]=true;
		list<int>::iterator it;
		//cout<<"for: "<<s<<" "<<parent<<endl;
		for(it=adjList[s].begin();it!=adjList[s].end();it++)
		{
			//cout<<"it: "<<*it<<" "<<visited[*it]<<" "<<parent<<endl;
			if(!visited[*it])
			{
				if(checkCycleUtil(*it,visited,s))
					return true;
			}
			else if(*it!=parent)
			{
				//cout<<*it<<" "<<parent<<"after ";
				return true;
			}
		}
		return false;
	}

	bool checkCycleUndir(){
		bool visited[v];
		for(int i=0;i<v;i++)
			visited[i]=false;

	    for(int i = 0; i < v; i++)
	    {
	    	if (!visited[i])
	    		if (checkCycleUtil(i,visited,-1))
	    			return true;
	    }
		return false;
	}

	bool checkCycleDirUtil(int s,bool visited[], bool recurStack[])
	{
		if(visited[s]==false)
		{
			visited[s]=true;
			recurStack[s]=true;
			list<int>::iterator it;
			for(it=adjList[s].begin();it!=adjList[s].end();it++)
			{
				if(!visited[*it] && checkCycleDirUtil(*it,visited,recurStack))
					return true;
				else if(recurStack[*it])
					return true;
			}
		}
		recurStack[s]=false;
		return false;
	}

	bool checkCycleDir(){
		bool visited[v];
		bool recurStack[v];
		for(int i=0;i<v;i++)
		{
			visited[i]=false;
			recurStack[i]=false;
		}

	    for(int i = 0; i < v; i++)
	    {
	        if (checkCycleDirUtil(i,visited,recurStack))
	            return true;
	    }
		return false;
	}

	void shortestPathUnweighted(int s)
	{
		queue<pair<int,int>> q;
		bool visited[v]={false};
		int count=0;

		q.push(pair<int,int>(s,count));
		visited[s]=true;

		while(!q.empty())
		{
			pair<int,int> t= q.front();
			s=t.first;
			cout<<t.first<<"-->"<<t.second<<endl;
			q.pop();

			list<int>::iterator it;
			for(it=adjList[s].begin();it!=adjList[s].end();it++)
			{
				if(!visited[*it])
				{
					count=t.second+1;
					q.push(pair<int,int>(*it,count));
					visited[*it]=true;
				}
			}
		}

	}

	void topologicalOrderUtil(int s, bool visited[], stack<int>& st){

		visited[s]=true;

		list<int>::iterator it;

		for(it=adjList[s].begin();it!=adjList[s].end();it++)
		{
			if(!visited[*it])
				topologicalOrderUtil(*it,visited,st);
		}

		st.push(s);
	}

	void topologicalOrder(){
		stack<int> s;
		bool visited[v]={false};

		for(int i=0;i<v;i++){
			if(!visited[i])
				topologicalOrderUtil(i,visited,s);
		}

		while(!s.empty())
		{
			cout<<s.top()<<" ";
			s.pop();
		}

	}

	//kahn's algorithm
	void topologicalOrderNew(){
		vector<int> in_degree(v,0);
		list<int>::iterator itr;

		for(int i=0;i<v;i++)
		{
			for(itr=adjList[i].begin();itr!=adjList[i].end();itr++)
				in_degree[*itr]++;
		}

		queue<int> q;

		for(int i=0;i<v;i++)
		{
			if(in_degree[i]==0)
				q.push(i);
		}

		vector<int> target;

		while(!q.empty()){
			int t=q.front();
			q.pop();
			target.push_back(t);
			for(itr=adjList[t].begin();itr!=adjList[t].end();itr++)
			{
				if(--in_degree[*itr]==0)
					q.push(*itr);
			}

		}

		cout<<"\nAlternate:  ";
		for(int i=0;i<target.size();i++)
			cout<< target[i]<<" ";

		cout<<endl;

	}



};

class Graph12{
	int v;
	list<int> * adjList;
public:

	Graph12(int V){
		this->v=V;
		adjList=new list<int>[2*V];
	}

	void addEdge(int V,int W, int weight){
		if(weight==2){
			adjList[V].push_back(v+V);
			adjList[v+V].push_back(W);
		}
		else
		{
			adjList[V].push_back(W);
		}
	}

	void printadjlist(){
		list<int>::iterator it;
		for(int i=0;i<2*v;i++)
		{
			cout<<"for "<<i<<": ";
			for(it=adjList[i].begin();it!=adjList[i].end();it++)
			{
				cout<<*it<<" ";
			}
			cout<<endl;
		}
	}

	int shortestPathUtil(int parent[],int s, int d){
		static int level=0;

		if(parent[s]==-1)
	    {
	        cout << "Shortest Path between " << s << " and "
	             << d << " is "  << s << " ";
	        return level;
	    }

		shortestPathUtil(parent, parent[s],d);

		level++;

		if(s<v)
			cout<<s<<" ";

		return level;
	}

	int shortestPath(int s, int d){
		 bool visited[2*v];
		 int parent[2*v];

		 for(int i=0;i<2*v;i++)
		 {
			 visited[i]=false;
			 parent[i]=-1;
		 }

		 queue<int> q;
		 q.push(s);
		 visited[s]=true;
		 list<int>::iterator it;

		 while(!q.empty()){
			 int curr=q.front();
			 q.pop();

			 if(curr==d)
				 return shortestPathUtil(parent,curr,d);

			 for(it=adjList[curr].begin();it!=adjList[curr].end();it++)
			 {
				 if(!visited[*it]){
					 visited[*it]=true;
					 parent[*it]=curr;
					 q.push(*it);
				 }
			 }

		 }
	}

};

class ALNode{
	int v;
	int weight;
public:
	ALNode(int V,int w){
		v=V;
		weight=w;
	}
	int getV(){
		return v;
	}
	int getWeight(){
		return weight;
	}
};

class WGraph{
	int v;
	list <ALNode> *adjList;

	void topologicalSortUtil(int s, bool visited[],stack<int>& st){

		visited[s]=true;
		list<ALNode>::iterator it;

		for(it=adjList[s].begin();it!=adjList[s].end();it++)
		{
			if(!visited[it->getV()])
				topologicalSortUtil(it->getV(),visited,st);
		}
		st.push(s);
	}

public:
	WGraph(int V){
		this->v=V;
		adjList=new list<ALNode>[V];
	}

	void addEdge(int V,int W,int weight){
		ALNode node(W,weight);
		adjList[V].push_back(node);
	}

	void shortestPath(int s){
	    bool *visited = new bool[v];
	    for (int i = 0; i < v; i++)
	    {
	        visited[i] = false;
	    }

		stack<int> st;

		int dst[v];

		for (int i = 0; i < v; i++)
		    if (visited[i] == false)
		         topologicalSortUtil(i, visited, st);

		for(int i=0;i<v;i++)
			dst[i]=INT_MAX;
		dst[s]=0;

		while(!st.empty()){
			int t=st.top();
			st.pop();
			//cout<<t<<" ";

			list<ALNode>::iterator it;

			if(dst[t]!=INT_MAX)
			{
				//cout<<t<<" ";
				for(it=adjList[t].begin();it!=adjList[t].end();it++)
				{
					if(dst[it->getV()]> (dst[t]+it->getWeight()))
					{
						//cout<<it->getV()<<" ";
						dst[it->getV()]=dst[t]+it->getWeight();
					}
				}
			}
		}

		for(int i=0;i<v;i++)
		{
			(dst[i]==INT_MAX)?cout<<"INF ":cout<<dst[i]<<" ";
		}
		cout<<endl;

	}



};

void checkConnected(int M[][col],int r, int c,bool visited[][col]){
	if(r>=row || c>=col || r<0 || c<0 || M[r][c]==0 || visited[r][c]==true)
		return;

	visited[r][c]=true;
	checkConnected(M,r-1,c,visited);
	checkConnected(M,r,c+1,visited);
	checkConnected(M,r,c-1,visited);
	checkConnected(M,r+1,c,visited);
}

int countIslands(int M[][col]){
	bool visited[row][col]={false};
	int count=0;

	for(int i=0;i<row;i++)
		for(int j=0;j<col;j++)
		{
			if(M[i][j]==1 && visited[i][j]==false)
			{
				checkConnected(M,i,j,visited);
				count++;
			}
		}

	return count;
}

int main() {

	//Directed
    Graph g(5);
    g.addEdge(0, 1);
    g.addEdge(0, 2);
    g.addEdge(1, 2);
    g.addEdge(2, 0);
    g.addEdge(2, 3);
    g.addEdge(3, 4);
    g.addEdge(4, 1);

    g.printadjlist();

    cout << "(BFS from vertex 2) \n";
    g.bfs(0);
    cout<<endl;
    cout << "(DFS from vertex 2) \n";
    g.dfs(0);
    cout<<endl;

    cout<<"Cycle Exists: "<<g.checkCycleDir()<<endl;
    g.shortestPathUnweighted(3);

    cout<<"*************************\n";
    //Undirected
    Graph g1(5);
    g1.addEdgeUndir(0, 2);
    g1.addEdgeUndir(0, 3);
    g1.addEdgeUndir(1, 0);
    g1.addEdgeUndir(2, 1);
    g1.addEdgeUndir(3, 4);

    cout << "(BFS from vertex 2) \n";
    g1.bfs(0);
    cout<<endl;
    cout << "(DFS from vertex 2) \n";
    g1.dfs(0);
    cout<<endl;

    cout<<"Cycle Exists: "<<g1.checkCycleUndir()<<endl;

    cout<<"*************************\n";

    //1-2 weighted
    Graph12 g2(4);
    g2.addEdge(0, 1, 2);
    g2.addEdge(0, 2, 2);
    g2.addEdge(1, 2, 1);
    g2.addEdge(1, 3, 1);
    g2.addEdge(2, 0, 1);
    g2.addEdge(2, 3, 2);
    g2.addEdge(3, 3, 2);

    g2.printadjlist();

    int src = 0, dest = 3;
    int sd=g2.shortestPath(src, dest);
    cout << "\nShortest Distance between " << src
         << " and " << dest << " is "
         << sd<<endl;

    cout<<"*************************\n";
    //acyclic directed graph
    Graph g3(6);
    g3.addEdge(5, 2);
    g3.addEdge(5, 0);
    g3.addEdge(4, 0);
    g3.addEdge(4, 1);
    g3.addEdge(2, 3);
    g3.addEdge(3, 1);
    cout << "Following is a Topological Sort of the given "
                "graph \n";

    // Function Call
    g3.topologicalOrder();
    g3.topologicalOrderNew();

    cout<<"*************************\n";

    //All weighted graph
    WGraph g4(6);
        g4.addEdge(0, 1, 5);
        g4.addEdge(0, 2, 3);
        g4.addEdge(1, 3, 6);
        g4.addEdge(1, 2, 2);
        g4.addEdge(2, 4, 4);
        g4.addEdge(2, 5, 2);
        g4.addEdge(2, 3, 7);
        g4.addEdge(3, 4, -1);
        g4.addEdge(4, 5, -2);

        cout << "Following are shortest distances from source \n";
        g4.shortestPath(1);

    cout<<"*************************\n";

    int M[][col] = { { 1, 1, 0, 0, 0 },
                     { 1, 1, 0, 0, 1 },
                     { 0, 0, 0, 1, 1 },
                     { 0, 0, 0, 0, 0 },
                     { 1, 1, 1, 0, 0 } };

    cout << "Number of islands is: " << countIslands(M);


	return 0;
}
