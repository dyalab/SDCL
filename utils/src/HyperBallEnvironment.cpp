#include "environment.h"

using namespace cv;

// defines a 2d configuration space based on a 2d image. 
HyperBallEnvironment::HyperBallEnvironment(pt lows, pt highs, pt start, pt goal, int N) {
    highs_ = highs; // joints limits.
    lows_ = lows;
    goal_ = goal;
    start_ = start;
    delta_= 0;
    inner_radius = 1;
    outter_radius = 2;
    N_ = N;
}

HyperBallEnvironment::~HyperBallEnvironment() {
    ;
}

bool HyperBallEnvironment::isStateValid(pt point) {
    // return whether state is valid
    double res = 0;
    for (int i = 0; i < N_; i++) {
        res += point[i] * point[i];
    }
    res = sqrt(res);
    if (res < inner_radius || res > outter_radius) return true;
    else return false;
}

double HyperBallEnvironment::penetrationDist(pt point, pt& close_point) {
    // return penetration depth of a point in collision. If point not in collision return -1.
    double res = 0;
    for (int i = 0; i < N_; i++) {
        res += point[i] * point[i];
    }
    res = sqrt(res);
    return std::min(res - inner_radius, outter_radius - res);
}
