#include "parser.h"

void parse(int argc, char ** argv, int* scene, double* duration, bool* use_training, bool* use_Gaussian) {
    double given_duration = 0;
    bool use_RRT = false; // default is run the PRM with inf planner.
    // std::cout << argc << std::endl;
    if (argc != 5) {
        std::cout << "Please provide a scene id (number 1-6), duration(in seconds), use_training(true or false), use_Gaussian(true or false)" << std::endl;
        exit(1);
    } else {
        *scene = std::atoi(argv[1]);
        *duration = std::atof(argv[2]);
        std::string training(argv[3]);
        std::string Gaussian(argv[4]);
        if (training == "true") *use_training = true;
        else *use_training = false;

        if (Gaussian == "true") *use_Gaussian = true;
        else *use_Gaussian = false;
    }
}
