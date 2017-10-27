#pragma once

#include <utility>
#include <vector>
#include <fstream>
#include <limits>

#include <CGAL/property_map.h>
#include "read_ply_points.hpp"

#include "CGAL.hpp"

class PlyInterpreter {
	std::vector<Point3>& groundPoints;
	std::vector<Point3>& nongroundPoints;


	bool warnedClassificationMissing = false;

	public:
	 PlyInterpreter (std::vector<Point3>& groundPoints, std::vector<Point3>& nongroundPoints)
	    : groundPoints (groundPoints), nongroundPoints(nongroundPoints)
	  { }
	  bool is_applicable (CGAL::Ply_reader& reader);
	  void process_line (CGAL::Ply_reader& reader);
	  bool flip_faces();
};
