#include "PlyInterpreter.hpp"

// Init and test if input file contains the right properties
bool PlyInterpreter::is_applicable(CGAL::Ply_reader& reader) {
	return reader.does_tag_exist<FT> ("x")
	      && reader.does_tag_exist<FT> ("y")
	      && reader.does_tag_exist<FT> ("z");
}

// Describes how to process one line (= one point object)
void PlyInterpreter::process_line(CGAL::Ply_reader& reader) {
	FT x = (FT)0., y = (FT)0., z = (FT)0;
	bool cameraPoint = false;
	const unsigned char CLASS_GROUND = 2;
	unsigned char classification = 2;

	reader.assign (x, "x");
	reader.assign (y, "y");
	reader.assign (z, "z");

	if (reader.does_tag_exist<unsigned char>("diffuse_red") &&
		reader.does_tag_exist<unsigned char>("diffuse_green") &&
		reader.does_tag_exist<unsigned char>("diffuse_blue")){
		unsigned char r = 0, g = 0, b = 0;

		reader.assign(r, "diffuse_red");
		reader.assign(g, "diffuse_green");
		reader.assign(b, "diffuse_blue");

		cameraPoint = (r == 0 && g == 0 && b == 255) ||
				 (r == 0 && g == 255 && b == 0) ||
				 (r == 255 && g == 0 && b == 0);
	}

	if (reader.does_tag_exist<unsigned char>("classification")){
		reader.assign(classification, "classification");
	}else if (!warnedClassificationMissing){
		std::cout << "WARNING: Points are missing the classification tag, will treat all points as ground.\n";
		warnedClassificationMissing = true;
	}

	if (!cameraPoint){
		Point3 p(x, y, z);

		if (classification == CLASS_GROUND){
			groundPoints.push_back(p);
		}else{
			nongroundPoints.push_back(p);
		}
	}
}

