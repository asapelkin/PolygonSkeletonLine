#include "stdafx.h"
#include <iostream>
#include <stdio.h>
#include "GraphLongestPath.h"
#include <algorithm>

using namespace std;
using namespace boost;

vector<int> getLongestPath(vector<Edge> graphedges, int n_nodes, int source)
{
	struct EdgeProperties {
		int weight;
	};

	typedef adjacency_list < vecS, vecS, directedS, no_property, EdgeProperties> Graph;

	//vector<Edge> graphedges(graph);
	//graphedges.reserve(graph.size());
	//for (const auto& it : graph)
	//	graphedges.push_back({ it.second, it.first });
		  
	int n_edges = graphedges.size();
//	int n_nodes = nodes.size();
	vector<int> weights(n_edges, 1);
		
	typedef adjacency_list < vecS, vecS, directedS,
		no_property, EdgeProperties> Graph;

	Graph g(graphedges.data(), graphedges.data() + n_edges, n_nodes);

	graph_traits < Graph >::edge_iterator ei, ei_end;
	property_map<Graph, int EdgeProperties::*>::type
		weight_pmap = get(&EdgeProperties::weight, g);
	int i = 0;
	for (boost::tie(ei, ei_end) = edges(g); ei != ei_end; ++ei, ++i)
		weight_pmap[*ei] = weights[i];

	std::vector<int> distance(n_nodes, (std::numeric_limits < short >::max)());
	std::vector<std::size_t> parent(n_nodes);
	for (i = 0; i < n_nodes; ++i)
		parent[i] = i;
	distance[source] = 0;
	
	bool r = bellman_ford_shortest_paths(g, int(n_nodes), weight_map(weight_pmap).distance_map(&distance[0]).predecessor_map(&parent[0]));

	auto farthestDistIt = std::max_element(distance.begin(), distance.end());
	int farthestNodeInd = std::distance(distance.begin(), farthestDistIt);

	vector<int> longestPath;
	longestPath.reserve(*farthestDistIt + 1);

	longestPath.push_back(farthestNodeInd);
	int it = farthestNodeInd;
	while (it != source)
	{
		it = parent[it];
		longestPath.push_back(it);
	} 

	return longestPath;
}