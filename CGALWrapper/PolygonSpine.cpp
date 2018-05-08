// CSharpCGAL_Wrapper.cpp : Defines the exported functions for the DLL application.
//

#include "stdafx.h"
#include "PolygonSpine.h"
#include "GraphLongestPath.h"


using namespace std;

#include <vector>
#include <sstream>

#include <boost/shared_ptr.hpp>
#include <CGAL/Cartesian.h>
typedef CGAL::Cartesian<float> K;

#include <CGAL/Polygon_2.h>
#include <CGAL/create_straight_skeleton_2.h>

typedef K::Point_2 Point_2;
typedef CGAL::Straight_skeleton_2<K> Ss;
typedef boost::shared_ptr<Ss> SsPtr;
 
#include <boost/geometry.hpp>
#include <boost/geometry/io/wkt/read.hpp> 
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
 
typedef boost::geometry::model::d2::point_xy<double> point_type;


string line2wkt(std::map<int, Point_2> nodesById, vector<int> nodes)
{
	std::ostringstream strs;
	strs << "LINESTRING (";
	int iddd = 0;	
	for (int num : nodes)
	{
		Point_2 point = nodesById[num];
		if (iddd > 0)		
			strs << "," ;		
		strs << point.x() << " " << point.y();
		iddd++;
	}	 
	strs << ")";
	return strs.str();
}
 

std::vector<Point_2> boostPol2CGALPol(const boost::geometry::model::polygon<point_type >& boostPolygon)
{
	std::vector<Point_2> cgalPol;
	for (auto it = boostPolygon.outer().begin() + 1; it < boostPolygon.outer().end(); it++)	
		cgalPol.push_back(Point_2(it->x(), it->y()));		  
	return cgalPol;
}


__declspec(dllexport) char* getPolygonSpine(const char* wktPolygon)
{	
	string wktPolygonStr(wktPolygon);
	
	boost::geometry::model::polygon<point_type> polygon;
	boost::geometry::read_wkt(wktPolygonStr, polygon);
	
	std::vector<Point_2> poly = boostPol2CGALPol(polygon);
	 
	SsPtr iss = CGAL::create_interior_straight_skeleton_2(poly.begin(), poly.end(), K());
 
	std::map<Point_2, int> nodesByPoint;
	std::map<int, Point_2> nodesById; 
	int id = 0;
	for (auto i = (*iss).vertices_begin(); i != (*iss).vertices_end(); ++i)  {
		/*if (i->is_contour())
			continue;*/
		nodesByPoint[i->point()] = id;
		nodesById[id] = i->point();
		id++;
	}
	
	std::vector<Edge> graph; // @todo размер !!
	  
	for (Ss::Halfedge_const_iterator i = (*iss).halfedges_begin(); i != (*iss).halfedges_end(); ++i)  {
		 
		if (i->is_border()) // пропускаем рёбра, примыкающие к границе полигона
			continue;
		if (i->opposite()->is_border())
			continue;

		int firstId = nodesByPoint[i->vertex()->point()];
		int secondId = nodesByPoint[i->opposite()->vertex()->point()];

		Edge edge(firstId, secondId);
		graph.push_back(edge); 
	} 

	vector<int> longestPath;
	for (auto it = nodesByPoint.begin(); it != nodesByPoint.end(); it++)
	{
		vector<int> curPath = getLongestPath(graph, nodesByPoint.size(), it->second);

		if (curPath.size() > longestPath.size())
			longestPath = curPath;
	}
		
	string linestr = line2wkt(nodesById, longestPath);

	

	// strcpy(resLine, linestr.c_str());

	char * resLine =  new char[linestr.length() + 1];
	 
	strcpy(resLine, linestr.c_str()); 

	return resLine;

	cout << "GEOMETRYCOLLECTION(" << endl;
	cout << wktPolygon << "," << endl;
	cout << resLine << endl;
	cout << ")" << endl;
}
 

