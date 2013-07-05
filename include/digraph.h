#pragma once
#include <hash_map>
#include <vector>
#include <cstdint>
#include <set>

#define ERadius 6371

struct Node {
	public:
		Node(double lat = 0, double lon = 0) : uses(0), lon_m(lon), lat_m(lat){}
		int uses;
		int cost;
		double lon_m;
		double lat_m;
};

float toSISpeed(int mph)
{
	return mph*0.44704;
}

double toRad(const double& degree)
{
	return 0.0174532925*degree;
};
//returns distance in meters
double dist2(const double& lat1, const double& lon1, const double& lat2, const double& lon2)
{
	double x = toRad(lon2 - lon1)*cos(toRad(lat1 + lat2)/2);
	double y = toRad(lat2 - lat1);
	return sqrt(x*x + y*y) * ERadius * 1000;
};

double dist2(const Node& a, const Node& b)
{
	return dist2(a.lat_m, a.lon_m, b.lat_m, b.lon_m);
};


struct Link {
public:
	Link(int _v, int _w, float _cost = 0) : v(_v), w(_w), cost(_cost){}
	float cost;
	int v;
	int w;
};


class Digraph {
public:
	typedef std::pair<uint64_t,float> AdjNode; //(id, cost)
	typedef std::vector<AdjNode> AdjListRow;
	typedef std::hash_map<uint64_t, AdjListRow> AdjList;
	AdjListRow::iterator adj_row_it;

	Digraph() {this->v = 0; this->e = 0;}
	Digraph(int V)
	{
		this->v = V;
		this->e = 0;
	}
	const int& V() {return this->all_nodes.size();}
	const int& E() {return this->e;}
	void addEdge(uint64_t v, uint64_t w, float cost)
	{
		this->adj[v].push_back(std::pair<uint64_t,float>(w, cost));
		this->e++;
		this->all_nodes.insert(v);
		this->all_nodes.insert(w);
	}

	std::set<uint64_t> AllNodes()
	{
		//std::vector<uint64_t> result;
		//for (adj_list_it=adj.begin(); adj_list_it!=adj.end(); ++adj_list_it)
		//{
		//	result.push_back(adj_list_it->first);
		//}
		//return result;
		return this->all_nodes;
	}
	//Returns the outgoiuing edges that are adjecent to the node
	AdjListRow& Adj(uint64_t v)
	{return this->adj[v];}
	Digraph reverse()
	{
		Digraph result(this->v);
		for (adj_list_it = adj.begin(); adj_list_it != adj.end(); ++adj_list_it)
		{
			uint64_t dest = adj_list_it->first;
			for (adj_row_it = adj_list_it->second.begin(); adj_row_it != adj_list_it->second.end(); ++adj_row_it)
			{
				result.addEdge(adj_row_it->first, dest, adj_row_it->second);
			}
		}
		return result;
	}



private:
	int v;
	int e;
	// maps a node to a vector of 2-tuples. Each tuple corresponds to an adjacent node
	// first element of the tuple contains the node id and the second constains link cost
	AdjList adj;
	AdjList::iterator adj_list_it;
	std::set<uint64_t> all_nodes;
	
};