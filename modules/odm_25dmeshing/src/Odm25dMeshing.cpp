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

		std::vector<Point3> groundPoints, nongroundPoints;

		loadPointCloud(inputFile, groundPoints, nongroundPoints);

		std::vector<Point3> bucket;
		preparePoints(groundPoints, bucket);
		preparePoints(nongroundPoints, bucket);

		buildMesh(bucket, outputFile);

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

void Odm25dMeshing::loadPointCloud(const std::string &inputFile, std::vector<Point3> &groundPoints, std::vector<Point3> &nongroundPoints){
	  PlyInterpreter interpreter(groundPoints, nongroundPoints);

	  std::ifstream in(inputFile);
	  if (!in || !CGAL::read_ply_custom_points (in, interpreter, Kernel())){
		  throw Odm25dMeshingException(
		  				"Error when reading points from:\n" + inputFile + "\n");
	  }

	  log << "Loaded " << groundPoints.size() << " ground points\n";
	  log << "Loaded " << nongroundPoints.size() << " non-ground points\n";

}

void Odm25dMeshing::preparePoints(const std::vector<Point3>& points, const std::vector<Point3> &destination){
	const unsigned int NEIGHBORS = 24;

	size_t pointCount = points.size();

	log << "Computing points average spacing... ";

	FT avgSpacing = CGAL::compute_average_spacing<Concurrency_tag>(
			points.begin(),
			points.end(),
			NEIGHBORS);

	log << avgSpacing << "\n";

	log << "Grid Z sampling... ";

	double gridStep = avgSpacing / 2.0;
	Kernel::Iso_cuboid_3 bbox = CGAL::bounding_box(points.begin(), points.end());
	Vector3 boxDiag = bbox.max() - bbox.min();

	int gridWidth = 1 + static_cast<unsigned>(boxDiag.x() / gridStep + 0.5);
	int gridHeight = 1 + static_cast<unsigned>(boxDiag.y() / gridStep + 0.5);

	#define KEY(i, j) (i * gridWidth + j)

	std::unordered_map<int, Point3> grid;

	for (size_t c = 0; c < pointCount; c++){
		const Point3 &p = points[c];
		Vector3 relativePos = p - bbox.min();
		int i = static_cast<int>((relativePos.x() / gridStep + 0.5));
		int j = static_cast<int>((relativePos.y() / gridStep + 0.5));

		if ((i >= 0 && i < gridWidth) && (j >= 0 && j < gridHeight)){
			int key = KEY(i, j);

			if (grid.find(key) == grid.end()){
				grid[key] = p;
			}else if (p.z() > grid[key].z()){
				grid[key] = p;
			}
		}
	}

	std::vector<Point3> gridPoints;
	for ( auto it = grid.begin(); it != grid.end(); ++it ){
		gridPoints.push_back(it->second);
	}

	pointCount = gridPoints.size();
	log << "sampled " << pointCount << " points\n";

	const double RETAIN_PERCENTAGE = std::min<double>(100., 100. * static_cast<double>(maxVertexCount) / static_cast<double>(pointCount));   // percentage of points to retain.

	log << "Performing weighted locally optimal projection simplification and regularization (retain: " << RETAIN_PERCENTAGE << "%, iterate: " << wlopIterations << ")" << "\n";

	CGAL::wlop_simplify_and_regularize_point_set<Concurrency_tag>(
			gridPoints.begin(),
			gridPoints.end(),
			std::back_inserter(destination),
			RETAIN_PERCENTAGE,
			-1,
			wlopIterations,
			true);
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



