#pragma once

// STL
#include <string>
#include <iostream>
#include <unordered_map>
#include <queue>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/impl/region_growing.hpp>
#include <pcl/surface/mls.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/mls.h>

#include "CGAL.hpp"

#include "Logger.hpp"
#include "PlyInterpreter.hpp"

class Odm25dMeshing {
public:
	Odm25dMeshing() :
			log(false) {};
	~Odm25dMeshing() {};

	/*!
	 * \brief   run     Runs the meshing functionality using the provided input arguments.
	 *                  For a list of accepted arguments, please see the main page documentation or
	 *                  call the program with parameter "-help".
	 * \param   argc    Application argument count.
	 * \param   argv    Argument values.
	 * \return  0       If successful.
	 */
	int run(int argc, char **argv);

private:

	/*!
	 * \brief parseArguments    Parses command line arguments.
	 * \param   argc    Application argument count.
	 * \param   argv    Argument values.
	 */
	void parseArguments(int argc, char** argv);

	/*!
	 * \brief loadPointCloud    Loads a PLY file with points from file.
	 */
	void loadPointCloud(const std::string &inputFile,
			pcl::PointCloud<pcl::PointXYZ>::Ptr groundPoints,
			pcl::PointCloud<pcl::PointXYZ>::Ptr nongroundPoints);

	/*!
	 * \brief preparePoints    Does gridding, smoothing, and places the results in destination
	 */
	void preparePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr groundPoints,
			pcl::PointCloud<pcl::PointXYZ>::Ptr nongroundPoints,
			pcl::PointCloud<pcl::PointXYZ>::Ptr destination);

	/*!
	 * \brief loadPointCloud    Builds a 2.5D mesh from loaded points
	 */
	void buildMesh(const std::vector<Point3>& points, const std::string &outputFile);

	/*!
	 * \brief printHelp     Prints help, explaining usage. Can be shown by calling the program with argument: "-help".
	 */
	void printHelp();

	Logger log;

	std::string inputFile = "";
	std::string outputFile = "odm_25dmesh.ply";
	std::string logFilePath = "odm_25dmeshing_log.txt";
	unsigned int maxVertexCount = 100000;
	unsigned int wlopIterations = 10;
};

class Odm25dMeshingException: public std::exception {

public:
	Odm25dMeshingException() :
			message("Error in Odm25dMeshing") {
	}
	Odm25dMeshingException(std::string msgInit) :
			message("Error in Odm25dMeshing:\n" + msgInit) {
	}
	~Odm25dMeshingException() throw () {
	}
	virtual const char* what() const throw () {
		return message.c_str();
	}

private:
	std::string message; /**< The error message **/
};
