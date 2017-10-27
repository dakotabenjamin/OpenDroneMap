#pragma once

#include <utility>
#include <vector>
#include <fstream>
#include <limits>

#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>

#include <CGAL/property_map.h>
#include "read_ply_points.hpp"
#include "CGAL.hpp"

class PlyInterpreter {
	pcl::PointCloud<pcl::PointXYZ>::Ptr groundPoints;
	pcl::PointCloud<pcl::PointXYZ>::Ptr nongroundPoints;

	bool warnedClassificationMissing = false;

	public:
	 PlyInterpreter (pcl::PointCloud<pcl::PointXYZ>::Ptr groundPoints, pcl::PointCloud<pcl::PointXYZ>::Ptr nongroundPoints)
	    : groundPoints (groundPoints), nongroundPoints(nongroundPoints)
	  { }
	  bool is_applicable (CGAL::Ply_reader& reader);
	  void process_line (CGAL::Ply_reader& reader);
	  bool flip_faces();
};
