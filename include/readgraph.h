#include <unordered_map>
#include <sstream>
#include "osmpbfreader.h"
#include "digraph.h"

#ifdef WGDAL
	#include <ogrsf_frmts.h>
#endif

#ifdef WBOOST
	#include <boost/graph/adjacency_list.hpp>
	#include <boost/graph/astar_search.hpp>
#endif

using namespace CanalTP;

// We keep every node and the how many times it is used in order to detect crossings
struct CNode {
	public:
		CNode(double lon = 0, double lat = 0) : uses(0), lon_m(lon), lat_m(lat){}
		int32_t uses;
		double lon_m;
		double lat_m;
};

struct Vertex {
	double lon_m;
	double lat_m;
};
struct Edge {
	float cost;
};

struct CWay
{
public:
	CWay(uint64_t _osmid, std::vector<uint64_t> _refs, Tags _tags) : osmid(_osmid), refs(_refs), tags(_tags){}
	std::vector<uint64_t> refs;
	uint64_t osmid;
	Tags tags;
};

typedef boost::adjacency_list<boost::listS, boost::vecS, boost::directedS, Vertex, Edge> Graph;
typedef Graph::vertex_descriptor VertexId;
typedef Graph::edge_descriptor EdgeId;

struct Routing {
	// Map that stores all the nodes read
	std::unordered_map<uint64_t, CNode> nodes;
	std::unordered_map<uint64_t, CNode>::iterator nodes_it;

	// Stores all the nodes of all the ways that are part of the road network
	std::vector< CWay > ways;
	std::vector< CWay >::iterator ways_it;

	// This method is called every time a CNode is read
	void node_callback(uint64_t osmid, double lon, double lat, const Tags &/*tags*/){
		this->nodes[osmid] = CNode(lon, lat);
	}

	// This method is called every time a Way is read
	void way_callback(uint64_t osmid, Tags &tags, const std::vector<uint64_t> &refs){
		// If the way is part of the road network we keep it
		// There are other tags that correspond to the street network, however for simplicity, we don't manage them
		// Homework: read more properties like oneways, bicycle lanes…
		Tags::iterator tag_it;
		tag_it = tags.find("highway");
		if(tag_it != tags.end()){
			if (tag_it->second == "footway" || tag_it->second == "cycleway" || tag_it->second == "pedestrian" || tag_it->second == "service" || tag_it->second == "steps" || tag_it->second == "crossing")
				return;
			ways.push_back(CWay(osmid, refs, tags));
		}
	}

	// Once all the ways and nodes are read, we count how many times a node is used to detect intersections
	void count_nodes_uses() 
	{
		std::cout << "Entered count_nodes_uses\n";
		for (std::vector< CWay >::iterator way = ways.begin();way != ways.end(); ++way)
		{
			for(std::vector<uint64_t>::iterator ref = way->refs.begin(); ref!= way->refs.end(); ++ref)
			{
				nodes_it = nodes.find(*ref);
				if (nodes_it != nodes.end())
					nodes_it->second.uses++;
				//else
					//std::cout << "Node " << *ref << " is not in the node map\n";
			}
			// make sure that the last node is considered as an extremity
			nodes_it = nodes.find(way->refs.back());
			if (nodes_it != nodes.end())
					nodes_it->second.uses++;
			//else
				//std::cout << "Final Node " << way->refs.back() << " is not in the node map\n";
		}
	}

	// Returns the source and target node of the edges
	std::vector< std::pair<uint64_t, uint64_t> > edges(){
		std::cout << "Entered edges\n";
		std::vector< std::pair<uint64_t, uint64_t> > result;
		for (std::vector< CWay >::iterator way = ways.begin();way != ways.end(); ++way){
			if(way->refs.size() > 0){
				uint64_t source = (way->refs)[0];
				for(size_t i = 1; i < way->refs.size(); ++i){
					uint64_t current_ref = (way->refs)[i];
					// If a node is used more than once, it is an intersection, hence it's a node of the road network graph
					nodes_it = nodes.find(current_ref);
					if (nodes_it == nodes.end())
						continue;
					if(nodes_it->second.uses > 1){
						// Homework: measure the length of the edge
						uint64_t target = current_ref;
						result.push_back(std::make_pair(source, target));
						source = target;
					}
				}
			}
		}
		return result;
	}

	Graph boost_graph(std::hash_map<VertexId, uint64_t> &ind2id, std::hash_map<uint64_t, VertexId> &id2ind)
	{
		Graph result;
		CNode  snode, tnode;
		Tags::iterator tag_it;
		float speed;
		std::string type;
		bool speed_is_input;
		int speed_count = 0;
		VertexId source_ind, target_ind;
		std::hash_map<uint64_t, VertexId>::iterator id2ind_it;
		for (std::vector< CWay >::iterator way = ways.begin();way != ways.end(); ++way){
			if(way->refs.size() > 0){
				tag_it = way->tags.find("maxspeed");
				if (tag_it != way->tags.end())
				{
					speed = toSISpeed(atoi(tag_it->second.c_str())); //in m/s
					speed_is_input = true;
				}
				else
				{
					speed_is_input = false;
					type = way->tags.find("highway")->second;
					if      (type == "motorway")		speed = toSISpeed(65);
					else if (type == "motorway_link")	speed = toSISpeed(45);
					else if (type == "trunk")			speed = toSISpeed(45);
					else if (type == "trunk_link")		speed = toSISpeed(35);
					else if (type == "primary")			speed = toSISpeed(45);
					else if (type == "secondary")		speed = toSISpeed(35);
					else if (type == "secondary_link")	speed = toSISpeed(30);
					else if (type == "residential")		speed = toSISpeed(25);
					else								speed = toSISpeed(35);					
				}
				uint64_t source = (way->refs)[0];
				nodes_it = nodes.find(source);
				if (nodes_it == nodes.end())
					continue;
				snode = nodes_it->second;
				id2ind_it = id2ind.find(source);
				if (id2ind_it == id2ind.end())
				{
					VertexId v_id = boost::add_vertex(result);
					result[v_id].lat_m = snode.lat_m;
					result[v_id].lon_m = snode.lon_m;
					id2ind[source] = v_id;
					ind2id[v_id] = source;
					source_ind = v_id;
				}
				else
					source_ind = id2ind_it->second;
				for(size_t i = 1; i < way->refs.size(); ++i){
					if (speed_is_input)
						speed_count++;
					uint64_t current_ref = (way->refs)[i];
					// If a node is used more than once, it is an intersection, hence it's a node of the road network graph
					nodes_it = nodes.find(current_ref);
					if (nodes_it == nodes.end())
						continue;
					tnode = nodes_it->second;					
					if(tnode.uses > 1){
						uint64_t target = current_ref;	
						id2ind_it = id2ind.find(target);
						if (id2ind_it == id2ind.end())
						{
							VertexId v_id = boost::add_vertex(result);
							result[v_id].lat_m = tnode.lat_m;
							result[v_id].lon_m = tnode.lon_m;
							id2ind[target] = v_id;
							ind2id[v_id] = target;
							target_ind = v_id;
						}
						else
							target_ind = id2ind_it->second;
						bool ok;
						EdgeId edge;
						boost::tie(edge, ok) = boost::add_edge(source_ind, target_ind, result);
						if (ok)
							result[edge].cost = dist2(snode.lat_m, snode.lon_m, tnode.lat_m, tnode.lon_m)/speed;
						source = target;
						snode = tnode;                    
					}
				}
			}
		}
		std::cout << "Number of links with speed: " << speed_count << "\n";
		return result;

	}

	Digraph digraph ()
	{
		CNode  snode, tnode;
		Tags::iterator tag_it;
		float speed;
		std::string type;
		bool speed_is_input;
		int speed_count = 0;
		Digraph result;
		for (std::vector< CWay >::iterator way = ways.begin();way != ways.end(); ++way){
			if(way->refs.size() > 0){
				tag_it = way->tags.find("maxspeed");
				if (tag_it != way->tags.end())
				{
					speed = toSISpeed(atoi(tag_it->second.c_str())); //in m/s
					speed_is_input = true;
				}
				else
				{
					speed_is_input = false;
					type = way->tags.find("highway")->second;
					if      (type == "motorway")		speed = toSISpeed(65);
					else if (type == "motorway_link")	speed = toSISpeed(45);
					else if (type == "trunk")			speed = toSISpeed(45);
					else if (type == "trunk_link")		speed = toSISpeed(35);
					else if (type == "primary")			speed = toSISpeed(45);
					else if (type == "secondary")		speed = toSISpeed(35);
					else if (type == "secondary_link")	speed = toSISpeed(30);
					else if (type == "residential")		speed = toSISpeed(25);
					else								speed = toSISpeed(35);					
				}
				uint64_t source = (way->refs)[0];
				nodes_it = nodes.find(source);
				if (nodes_it == nodes.end())
					continue;
				snode = nodes_it->second;
				for(size_t i = 1; i < way->refs.size(); ++i){
					if (speed_is_input)
						speed_count++;
					uint64_t current_ref = (way->refs)[i];
					// If a node is used more than once, it is an intersection, hence it's a node of the road network graph
					nodes_it = nodes.find(current_ref);
					if (nodes_it == nodes.end())
						continue;
					tnode = nodes_it->second;					
					if(tnode.uses > 1){
						uint64_t target = current_ref;						
						result.addEdge(source, target, dist2(snode.lat_m, snode.lon_m, tnode.lat_m, tnode.lon_m)/speed);
						source = target;
						snode = tnode;                    
					}
				}
			}
		}
		std::cout << "Number of links with speed: " << speed_count << "\n";
		return result;
	}
#ifdef WGDAL
	int toShape(std::string file_path) {
		const char *pszDriverName = "ESRI Shapefile";
		OGRSFDriver *poDriver;
		OGRDataSource *poDS;
		OGRRegisterAll();
		poDriver = OGRSFDriverRegistrar::GetRegistrar()->GetDriverByName(
				pszDriverName );
		if( poDriver == NULL )
		{
			printf( "%s driver not available.\n", pszDriverName );
			return 1;
		}
		poDS = poDriver->CreateDataSource( file_path.c_str(), NULL );
		if (poDS == NULL) 
		{
			printf("Creation of output shapefile failed.\n");
			return 2;
		}

		OGRLayer *poLayer;
		std::cout << "creating ref\n";
		OGRSpatialReference ref(NULL);
		int res = ref.SetWellKnownGeogCS( "EPSG:4326" );
		std::cout << "Import result: " << res << "\n";
		if (res != OGRERR_NONE) return 10;
		std::cout << "Creating layer\n";
		poLayer = poDS->CreateLayer("Edges", &ref, wkbLineString, NULL);
		if (poLayer == NULL) return 3;
		OGRFieldDefn nameField( "Name", OFTString ); nameField.SetWidth(32);
		if (poLayer->CreateField(&nameField) != OGRERR_NONE) return 4;
		OGRFieldDefn idField( "Id", OFTString); idField.SetWidth(32);
		if (poLayer->CreateField(&idField) != OGRERR_NONE) return 4;
		OGRFieldDefn typeField( "Type", OFTString ); nameField.SetWidth(32);
		if (poLayer->CreateField(&typeField) != OGRERR_NONE) return 4;
		OGRFieldDefn speedField( "Speed", OFTString ); nameField.SetWidth(32);
		if (poLayer->CreateField(&speedField) != OGRERR_NONE) return 4;

		//add features

		OGRFeature *poFeature;
		OGRLineString geo;
		Tags tags;
		Tags::iterator tag_it;
		CNode snode, tnode;
		int count = 0;
		std::ostringstream ss;
		std::cout << "Adding Edges to the Shapefile\n";
		for (std::vector< CWay >::iterator way = ways.begin();way != ways.end(); ++way){
			if (count%10000 == 0)
				std::cout << count << "\n";
			count++;
			if(way->refs.size() > 0){
				tags = way->tags;
				tag_it = tags.find("highway");
				poFeature = OGRFeature::CreateFeature( poLayer->GetLayerDefn() );				
				poFeature->SetField("Type", tag_it->second.c_str());
				ss.clear();
				ss.str("");
				ss << way->osmid;
				poFeature->SetField("Id", ss.str().c_str());
				tag_it = tags.find("name");
				if (tag_it != tags.end())
					poFeature->SetField("Name", tag_it->second.c_str());
				else 
					poFeature->SetField("Name", "");
				tag_it = tags.find("maxspeed");
				if (tag_it != tags.end())
					poFeature->SetField("Speed", tag_it->second.c_str());
				else 
					poFeature->SetField("Speed", "");
				uint64_t source = (way->refs)[0];
				nodes_it = nodes.find(source);
				if (nodes_it == nodes.end())
					continue;
				snode = nodes_it->second;		
				geo.empty();
				bool add_source = true;
				for(size_t i = 1; i < way->refs.size(); ++i){
					if (add_source)
						geo.addPoint(snode.lon_m, snode.lat_m);
					add_source = false;
					uint64_t current_ref = (way->refs)[i];
					nodes_it = nodes.find(current_ref);
					if (nodes_it == nodes.end())
						continue;
					// If a node is used more than once, it is an intersection, hence it's a node of the road network graph
					tnode = nodes_it->second;
					geo.addPoint(tnode.lon_m, tnode.lat_m);
					if(tnode.uses > 1){
						uint64_t target = current_ref;						
						source = target;
						snode = tnode;
						poFeature->SetGeometry(&geo);
						if( poLayer->CreateFeature( poFeature ) != OGRERR_NONE ) return 5;
						geo.empty();
						add_source = true;
					}

				}

			}
			OGRFeature::DestroyFeature( poFeature );
		}
		OGRDataSource::DestroyDataSource( poDS );
		return 0;
	}
#endif

	// We don't care about relations
	void relation_callback(uint64_t /*osmid*/, const Tags &/*tags*/, const References & /*refs*/){}
};