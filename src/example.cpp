#include "readgraph.h"


int main(int argc, char** argv) {
	if(argc < 2) {
		std::cout << "Usage: " << argv[0] << " <file.osm.pbf>" << std::endl;
		return 1;
	}
	// Let's read that file !
	Routing routing;
	read_osm_pbf(argv[1], routing);
	std::cout << "We read " << routing.nodes.size() << " nodes and " << routing.ways.size() << " ways" << std::endl;
	routing.count_nodes_uses();    


	std::cout << "The routing graph has " << routing.edges().size() << " edges" << std::endl;

	Digraph dg = routing.digraph();
	Digraph dg_r = dg.reverse();
#if defined WDAL
	if (argc > 2)
	{
		remove(argv[2]);
		int res = routing.toShape(argv[2]);
		std::cout << "toShape returned " << res << "\n";
	}
#endif
	std::cout << "The routing graph has " << dg.E() << " edges" << std::endl;
	std::cout << "The routing graph has " << dg.V() << " nodes" << std::endl;
	std::cout << "The reverse routing graph has " << dg_r.E() << " edges" << std::endl;
	std::cout << "The reverse routing graph has " << dg_r.V() << " nodes" << std::endl;
	std::hash_map<VertexId , uint64_t> ind2id;
	std::hash_map<uint64_t, VertexId > id2ind;
	Graph boost_graph = routing.boost_graph(ind2id, id2ind);
	std::cout << "Boost graph has " << boost::num_edges(boost_graph) << " edges\n";
	std::cout << "Boost graph has " << boost::num_vertices(boost_graph) << " vertices\n";
	return 0;
}

