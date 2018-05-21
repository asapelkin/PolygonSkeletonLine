 
#include <boost/config.hpp>
#include <iostream>
#include <fstream>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/bellman_ford_shortest_paths.hpp>

typedef std::pair<int, int> Edge;

std::vector<int> getLongestPath(const std::vector<Edge>& graphedges, const std::vector<double>& weights, int n_nodes, int source);