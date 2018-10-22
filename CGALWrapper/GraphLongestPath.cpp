#include "stdafx.h"
#include <iostream>
#include <stdio.h>
#include "GraphLongestPath.h"
#include <algorithm>
#include <stdexcept>

using namespace std;
using namespace boost;

 
void getLongestPath(const vector<Edge>& graphedges, const vector<double>& weights, int n_nodes, int source, list<int>& res, double& resPathLength)
{
	struct EdgeProperties {
		double weight;
	};

	typedef adjacency_list < vecS, vecS, directedS, no_property, EdgeProperties> Graph;

	int n_edges = graphedges.size();

	Graph g(graphedges.data(), graphedges.data() + n_edges, n_nodes);

	graph_traits < Graph >::edge_iterator ei, ei_end;
	property_map<Graph, double EdgeProperties::*>::type
		weight_pmap = get(&EdgeProperties::weight, g);
	int i = 0;
	for (boost::tie(ei, ei_end) = edges(g); ei != ei_end; ++ei, ++i)
		weight_pmap[*ei] = weights[i];

	std::vector<double> distance(n_nodes, (std::numeric_limits < double >::max)());
	std::vector<std::size_t> parent(n_nodes);
	for (i = 0; i < n_nodes; ++i)
		parent[i] = i;
	distance[source] = 0;

	bool r;

	try     {
		r = bellman_ford_shortest_paths(g, int(n_nodes), weight_map(weight_pmap).distance_map(&distance[0]).predecessor_map(&parent[0]));
	}
	catch (std::exception)
	{
		throw std::runtime_error("Error in corridor processing");
	}

	if (!r)
		throw std::runtime_error("Error in corridor processing");

	auto farthestDistIt = std::max_element(distance.begin(), distance.end());
	int farthestNodeInd = std::distance(distance.begin(), farthestDistIt);

	res.clear();

	res.push_back(farthestNodeInd);
	int it = farthestNodeInd;
	while (it != source)
	{
		it = parent[it];
		res.push_back(it);
		if (res.size() > n_nodes)
			throw std::runtime_error("Error in corridor processing");
	}
	resPathLength = distance[farthestNodeInd];
}
