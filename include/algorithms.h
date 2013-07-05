#include <vector>
#include <queue>
#include "digraph.h"
#include <cfloat>
#include <boost/heap/fibonacci_heap.hpp>

struct NodeCost {
	uint64_t id;
	float cost;
	bool operator< (const NodeCost& a) 
	{
		return cost > a.cost;
	}
};

bool operator< (const NodeCost& a, const NodeCost& b) 
{
	return a.cost > b.cost;
};


float Dist(Digraph& g, const uint64_t& s, const uint64_t& d)
{
	typedef boost::heap::fibonacci_heap<NodeCost> Heap;
	Heap q;
	std::hash_map<uint64_t,typename  Heap::handle_type> references;
	std::hash_map<uint64_t,float> dist;
	std::hash_map<uint64_t,uint64_t> predecessor;
	NodeCost n;

	std::set<uint64_t> vn = g.AllNodes();
	for (std::set<uint64_t>::iterator it = vn.begin(); it!=vn.end(); ++it)
	{
		n.id = *it;
		n.cost = FLT_MAX;
		if (n.id == s)
			n.cost = 0.0;		
		references[n.id] = q.push(n);
		dist[n.id] = n.cost;
		predecessor[n.id] = -1;
	}
	while (!q.empty())
	{
		float alt;
		NodeCost u = q.top();
		q.pop();
		if (u.id == d)
			break;
		Digraph::AdjListRow ar = g.Adj(u.id);
		for (Digraph::AdjListRow::iterator v =  ar.begin(); v!=ar.end(); ++v)
		{
			alt = u.cost + v->second;
			if (alt < dist[v->first]) {
				dist[v->first] = alt;
				Heap::handle_type pt = references[v->first];
				(*pt).cost = alt;
				q.update(pt);
				predecessor[v->first] = u.id;
			}
		}
	}
	return dist[d];
}