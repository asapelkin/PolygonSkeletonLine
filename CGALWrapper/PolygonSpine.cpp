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

#include <CGAL/Polygon_2.h>
#include <CGAL/create_straight_skeleton_2.h>

#include <boost/geometry.hpp>
#include <boost/geometry/io/wkt/read.hpp> 
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
 
typedef CGAL::Cartesian<float>::Point_2 Point_2;
typedef CGAL::Straight_skeleton_2<CGAL::Cartesian<float>> StraightSkeleton;
typedef boost::geometry::model::d2::point_xy<double> point_type;
typedef boost::geometry::model::polygon<point_type> BoostPolygon;


std::vector<Point_2> boostPol2CGALPol(const BoostPolygon & boostPolygon)
{
	std::vector<Point_2> cgalPol;
	for (auto it = boostPolygon.outer().begin() + 1; it < boostPolygon.outer().end(); it++)
		cgalPol.push_back(Point_2(it->x(), it->y()));
	return cgalPol;
}

string skeleton2wkt(boost::shared_ptr<const StraightSkeleton> skeleton)
{
	std::ostringstream strs;
	strs << "MULTILINESTRING ("; 
	bool flag = true;
	for (StraightSkeleton::Halfedge_const_iterator i = (*skeleton).halfedges_begin(); i != (*skeleton).halfedges_end(); ++i)  {
		if (i->is_border()) // пропускаем рёбра, примыкающие к границе полигона
			continue;
		if (i->opposite()->is_border())
			continue;

		if (flag) 
			flag = false;		
		else
			strs << ",";

		strs << "("<< i->vertex()->point().x() << "  " << i->vertex()->point().y() << ",   " << i->opposite()->vertex()->point().x() << "  " << i->opposite()->vertex()->point().y() <<")" << endl;

	}
	strs << ")";
	return  strs.str();
}

string line2wkt(const std::map<int, Point_2>& nodesById, const vector<int>& nodes)
{
	std::ostringstream strs;
	strs << "LINESTRING (";
	
	for (int i(0); i < nodes.size(); i++)
	{  
		const Point_2& point = nodesById.at(nodes[i]);		 
		if (i != 0)
			strs << "," ;		
		strs << point.x() << " " << point.y();		
	}	 
	strs << ")";
	return strs.str();
}
 

void preprocPolygon(BoostPolygon& polygon)
{
	BoostPolygon res;
	std::vector<point_type> points;
	points.reserve(polygon.outer().size());
	points.push_back(point_type(polygon.outer().front().x(), polygon.outer().front().y()));
	for (auto it = polygon.outer().begin() + 1; it < polygon.outer().end()-1; it++)
	{
		const point_type& prev = *(it-1);
		const point_type& cur = *it;
		const point_type& next = *(it + 1);
		auto ab = boost::geometry::distance(prev, cur);
		auto bc = boost::geometry::distance(cur, next);
		auto ac = boost::geometry::distance(prev, next);
		
		if ( ab + bc == ac)
			continue;	

		points.push_back(point_type(it->x(), it->y()));
	}
	points.push_back(point_type(polygon.outer().back().x(), polygon.outer().back().y()));
	boost::geometry::assign_points(res, points);
	polygon = res;
}


__declspec(dllexport) char* getPolygonSpine(const char* wktPolygon)
{	 
	string wktPolygonStr(wktPolygon);
	
	BoostPolygon polygon;
	boost::geometry::read_wkt(wktPolygonStr, polygon);
	 
	preprocPolygon(polygon);
 
	std::vector<Point_2> poly = boostPol2CGALPol(polygon);
	 
	boost::shared_ptr<StraightSkeleton> iss;

	try
	{
		iss = CGAL::create_interior_straight_skeleton_2(poly.begin(), poly.end(), CGAL::Cartesian<float>());
	}
	catch (std::exception)
	{
		throw std::runtime_error("Error in the straigh skeleton algorithm");
	}
 
	std::map<Point_2, int> nodesByPoint;
	std::map<int, Point_2> nodesById; 
	int id = 0;
	for (auto i = (*iss).vertices_begin(); i != (*iss).vertices_end(); ++i)  {		
		// если точка граничная (принадлежит полигону), то отбрасываем её
		if (find(poly.begin(), poly.end(), i->point()) != poly.end())
			continue;
		
		nodesByPoint[i->point()] = id;
		nodesById[id] = i->point();
		id++;
	}
	
	std::vector<Edge> graph; // @todo размер !!
	  
	for (StraightSkeleton::Halfedge_const_iterator i = (*iss).halfedges_begin(); i != (*iss).halfedges_end(); ++i)  {	 

		if (!nodesByPoint.count(i->vertex()->point()) 
		 || !nodesByPoint.count(i->opposite()->vertex()->point()))
			continue;

		int firstId = nodesByPoint[i->vertex()->point()];
		int secondId = nodesByPoint[i->opposite()->vertex()->point()];

		Edge edge(firstId, secondId);
		graph.push_back(edge);

	}
	
	cout << "GEOMETRYCOLLECTION(" << endl;
	cout << wktPolygon << "," << endl;
 
	cout << "MULTILINESTRING (";
	bool flag = true;
	for (const auto& edge : graph)
	{
	if (flag)
	flag = false;
	else
	cout << ",";

	auto p1 = nodesById[edge.first];
	auto p2 = nodesById[edge.second];

	cout << "(" << p1.x() << "  " << p1.y() << ",   " << p2.x() << "  " << p2.y() << ")" << endl;

	}
	cout << ")";
 
	cout << ")" << endl;

	vector<int> longestPath;
	for (auto it = nodesByPoint.begin(); it != nodesByPoint.end(); it++)
	{
		vector<int> curPath = getLongestPath(graph, nodesByPoint.size(), it->second);

		if (curPath.size() > longestPath.size())
			longestPath = curPath;
	}
		
	string linestr = line2wkt(nodesById, longestPath);
	 
	char * resLine =  new char[linestr.length() + 1];
	 
	strcpy(resLine, linestr.c_str()); 

	
	
	cout << "GEOMETRYCOLLECTION(" << endl;
	cout << wktPolygon << "," << endl;
	cout << linestr << endl;
/*
	cout << "MULTILINESTRING (";
	bool flag = true;
	for (const auto& edge : graph)
	{
		if (flag)
			flag = false;
		else
			cout << ",";

		auto p1 = nodesById[edge.first];
		auto p2 = nodesById[edge.second];

		cout << "(" << p1.x() << "  " << p1.y() << ",   " << p2.x() << "  " << p2.y() << ")" << endl;

	}
	cout << ")";
*/
	cout << ")" << endl;

	return resLine;
}
 

