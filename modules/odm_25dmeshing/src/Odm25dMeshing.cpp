#include "Odm25dMeshing.hpp"


int Odm25dMeshing::run(int argc, char **argv) {
	log << logFilePath << "\n";

	// If no arguments were passed, print help and return early.
	if (argc <= 1) {
		printHelp();
		return EXIT_SUCCESS;
	}

	try {

		parseArguments(argc, argv);

		pcl::PointCloud<pcl::PointXYZ>::Ptr groundPoints(new pcl::PointCloud<pcl::PointXYZ>()),
											nongroundPoints(new pcl::PointCloud<pcl::PointXYZ>()),
											bucket(new pcl::PointCloud<pcl::PointXYZ>());

		loadPointCloud(inputFile, groundPoints, nongroundPoints);

		preparePoints(groundPoints, nongroundPoints, bucket);
//
//		buildMesh(bucket, outputFile);

	} catch (const Odm25dMeshingException& e) {
		log.setIsPrintingInCout(true);
		log << e.what() << "\n";
		log.printToFile(logFilePath);
		log << "For more detailed information, see log file." << "\n";
		return EXIT_FAILURE;
	} catch (const std::exception& e) {
		log.setIsPrintingInCout(true);
		log << "Error in OdmMeshing:\n";
		log << e.what() << "\n";
		log.printToFile(logFilePath);
		log << "For more detailed information, see log file." << "\n";
		return EXIT_FAILURE;
	}

	log.printToFile(logFilePath);

	return EXIT_SUCCESS;
}

void Odm25dMeshing::preparePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr groundPoints,
		pcl::PointCloud<pcl::PointXYZ>::Ptr nongroundPoints,
		pcl::PointCloud<pcl::PointXYZ>::Ptr destination){

	const int K = 24,
			  MIN_CLUSTER_SIZE = 100;

	// Extract nonground normals
	pcl::PointCloud<pcl::Normal>::Ptr nongroundNormals (new pcl::PointCloud<pcl::Normal>);
	pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud (nongroundPoints);
	ne.setSearchMethod(tree);
	ne.setRadiusSearch (0.1);
	ne.compute(*nongroundNormals);

	std::vector <pcl::PointIndices> clusters;
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize (MIN_CLUSTER_SIZE);
	reg.setMaxClusterSize (nongroundPoints->size());
	reg.setSearchMethod (tree);
	reg.setNumberOfNeighbours (K);
	reg.setInputCloud (nongroundPoints);
	reg.setInputNormals (nongroundNormals);
	reg.setSmoothnessThreshold (45 / 180.0 * M_PI);
	reg.setCurvatureThreshold (1);
	reg.setResidualTestFlag(true);
	reg.extract (clusters);

	// TODO: remove
	pcl::io::savePLYFile("colored.ply", *reg.getColoredCloud());

	// TODO: smooth ground points via MLS

	// TODO: for each cluster
	// - Smooth via MLS
	// - Compute 2D convex
	// - Remove points that fall within convex hull (in ground dataset, via crophull)
	// - For each convex hull point, find nearest neighbor in ground dataset, use Z value to create a skirt
	// - merge into bucket

	// TODO: triangulate

//	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
//
//	mls.setComputeNormals (true);
//
//	// Set parameters
//	mls.setInputCloud (cloud);
//	mls.setPolynomialFit (true);
//	mls.setSearchMethod (tree);
//	mls.setSearchRadius (0.03);
//
//	// Reconstruct
//	mls.process (mls_points);
}

void Odm25dMeshing::parseArguments(int argc, char **argv) {
	for (int argIndex = 1; argIndex < argc; ++argIndex) {
		// The argument to be parsed.
		std::string argument = std::string(argv[argIndex]);

		if (argument == "-help") {
			printHelp();
			exit(0);
		} else if (argument == "-verbose") {
			log.setIsPrintingInCout(true);
		} else if (argument == "-maxVertexCount" && argIndex < argc) {
            ++argIndex;
            if (argIndex >= argc) throw Odm25dMeshingException("Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
            std::stringstream ss(argv[argIndex]);
            ss >> maxVertexCount;
            if (ss.bad()) throw Odm25dMeshingException("Argument '" + argument + "' has a bad value (wrong type).");
            maxVertexCount = std::max<unsigned int>(maxVertexCount, 0);
            log << "Vertex count was manually set to: " << maxVertexCount << "\n";
		} else if (argument == "-wlopIterations" && argIndex < argc) {
			++argIndex;
			if (argIndex >= argc) throw Odm25dMeshingException("Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
			std::stringstream ss(argv[argIndex]);
			ss >> wlopIterations;
			if (ss.bad()) throw Odm25dMeshingException("Argument '" + argument + "' has a bad value (wrong type).");

			wlopIterations = std::min<unsigned int>(1000, std::max<unsigned int>(wlopIterations, 1));
			log << "WLOP iterations was manually set to: " << wlopIterations << "\n";
		} else if (argument == "-inputFile" && argIndex < argc) {
			++argIndex;
			if (argIndex >= argc) {
				throw Odm25dMeshingException(
						"Argument '" + argument
								+ "' expects 1 more input following it, but no more inputs were provided.");
			}
			inputFile = std::string(argv[argIndex]);
			std::ifstream testFile(inputFile.c_str(), std::ios::binary);
			if (!testFile.is_open()) {
				throw Odm25dMeshingException(
						"Argument '" + argument	+ "' has a bad value. (file not accessible)");
			}
			testFile.close();
			log << "Reading point cloud at: " << inputFile << "\n";
		} else if (argument == "-outputFile" && argIndex < argc) {
			++argIndex;
			if (argIndex >= argc) {
				throw Odm25dMeshingException(
						"Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
			}
			outputFile = std::string(argv[argIndex]);
			std::ofstream testFile(outputFile.c_str());
			if (!testFile.is_open()) {
				throw Odm25dMeshingException(
						"Argument '" + argument + "' has a bad value.");
			}
			testFile.close();
			log << "Writing output to: " << outputFile << "\n";
		} else if (argument == "-logFile" && argIndex < argc) {
			++argIndex;
			if (argIndex >= argc) {
				throw Odm25dMeshingException(
						"Argument '" + argument
								+ "' expects 1 more input following it, but no more inputs were provided.");
			}
			logFilePath = std::string(argv[argIndex]);
			std::ofstream testFile(outputFile.c_str());
			if (!testFile.is_open()) {
				throw Odm25dMeshingException(
						"Argument '" + argument + "' has a bad value.");
			}
			testFile.close();
			log << "Writing log information to: " << logFilePath << "\n";
		} else {
			printHelp();
			throw Odm25dMeshingException("Unrecognised argument '" + argument + "'");
		}
	}
}

void Odm25dMeshing::loadPointCloud(const std::string &inputFile,
		pcl::PointCloud<pcl::PointXYZ>::Ptr groundPoints,
		pcl::PointCloud<pcl::PointXYZ>::Ptr nongroundPoints){
	  PlyInterpreter interpreter(groundPoints, nongroundPoints);

	  std::ifstream in(inputFile);
	  if (!in || !CGAL::read_ply_custom_points (in, interpreter, Kernel())){
		  throw Odm25dMeshingException(
		  				"Error when reading points from:\n" + inputFile + "\n");
	  }

	  log << "Loaded " << groundPoints->size() << " ground points\n";
	  log << "Loaded " << nongroundPoints->size() << " non-ground points\n";

}

void Odm25dMeshing::buildMesh(const std::vector<Point3>& points, const std::string &outputFile){
	size_t pointCount = points.size();
	log << "Vertex count is " << pointCount << "\n";

	if (pointCount < 3){
		throw Odm25dMeshingException("Not enough points");
	}

	typedef CDT::Point cgalPoint;
	std::vector< std::pair<cgalPoint, size_t > > pts;
	try{
		pts.reserve(pointCount);
	} catch (const std::bad_alloc&){
		throw Odm25dMeshingException("Not enough memory");
	}

	for (size_t i = 0; i < pointCount; ++i){
		pts.push_back(std::make_pair(cgalPoint(points[i].x(), points[i].y()), i));
	}

	log << "Computing delaunay triangulation... ";

	CDT cdt;
	cdt.insert(pts.begin(), pts.end());

	unsigned int numberOfTriangles = static_cast<unsigned >(cdt.number_of_faces());
	unsigned int triIndexes = numberOfTriangles*3;

	if (numberOfTriangles == 0) throw Odm25dMeshingException("No triangles in resulting mesh");

	log << numberOfTriangles << " triangles\n";

	std::vector<float> vertices;
	std::vector<int> vertexIndices;

	try{
		vertices.reserve(pointCount);
		vertexIndices.reserve(triIndexes);
	} catch (const std::bad_alloc&){
		throw Odm25dMeshingException("Not enough memory");
	}


	log << "Saving mesh to file.\n";

	std::filebuf fb;
	fb.open(outputFile, std::ios::out);
	std::ostream os(&fb);

	os << "ply\n"
	   << "format ascii 1.0\n"
	   << "element vertex " << pointCount << "\n"
	   << "property float x\n"
	   << "property float y\n"
	   << "property float z\n"
	   << "element face " << numberOfTriangles << "\n"
	   << "property list uchar int vertex_index\n"
	   << "end_header\n";

	for (size_t i = 0; i < pointCount; ++i){
		os << points[i].x() << " " << points[i].y() << " " << points[i].z() << std::endl;
	}

	for (CDT::Face_iterator face = cdt.faces_begin(); face != cdt.faces_end(); ++face) {
		os << 3 << " " << face->vertex(0)->info() << " " << face->vertex(1)->info() << " " << face->vertex(2)->info() << std::endl;
	}

	fb.close();

	log << "Successfully wrote mesh to: " << outputFile << "\n";
}

void Odm25dMeshing::printHelp() {
	bool printInCoutPop = log.isPrintingInCout();
	log.setIsPrintingInCout(true);

	log << "Usage: odm_25dmeshing -inputFile [plyFile] [optional-parameters]\n";
	log << "Create a 2.5D mesh from an oriented, classified point cloud (points with normals, classification and heightaboveground property) using a constrained delaunay triangulation. "
		<< "The program requires a path to an input PLY point cloud file, all other input parameters are optional.\n\n";

	log << "	-inputFile	<path>	to PLY point cloud\n"
		<< "	-outputFile	<path>	where the output PLY 2.5D mesh should be saved (default: " << outputFile << ")\n"
		<< "	-logFile	<path>	log file path (default: " << logFilePath << ")\n"
		<< "	-verbose	whether to print verbose output (default: " << (printInCoutPop ? "true" : "false") << ")\n"
		<< "	-maxVertexCount	<0 - N>	Maximum number of vertices in the output mesh. The mesh might have fewer vertices, but will not exceed this limit. (default: " << maxVertexCount << ")\n"
		<< "	-wlopIterations	<1 - 1000>	Iterations of the Weighted Locally Optimal Projection (WLOP) simplification algorithm. Higher values take longer but produce a smoother mesh. (default: " << wlopIterations << ")\n"

		<< "\n";

	log.setIsPrintingInCout(printInCoutPop);
}



