#include "ErrorEstimator.h"

#include <iostream>
#include <fstream>
#include <string>

int main(int argc, char** argv) {
	std::fstream in(argv[1]);
	if (!in.good())
		return false;

	std::string path_rs_configuration;
	std::string path_profile;
	std::string path_optitrack_calib;

	in >> path_rs_configuration;
	in >> path_profile;
	in >> path_optitrack_calib;
	
	ee::ErrorEstimator* error_estimator = new ee::ErrorEstimator(path_rs_configuration, path_profile, path_optitrack_calib);
	if (error_estimator->initialize())
		error_estimator->run();


	return 0;
}