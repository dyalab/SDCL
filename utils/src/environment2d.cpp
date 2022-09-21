#include "environment.h"

using namespace cv;

// defines a 2d configuration space based on a 2d image. 
Environment2D::Environment2D(const std::string path, pt lows, pt highs, pt start, pt goal) {
    highs_ = highs; // joints limits.
    lows_ = lows;
    goal_ = goal;
    start_ = start;
    N_ = start.size();
    delta_= 0;
    image = imread(path, IMREAD_COLOR);
    if(image.empty())
    {
        std::cout << "Could not read the image. " << std::endl;
        exit(1);
    }
    row = (double)image.rows;
    col = (double)image.cols;
}

Environment2D::~Environment2D() {
    image.release();
}

bool Environment2D::isStateValid(pt point) {
    // return whether state is valid
    // map points from given joint limits to point in image
    int point_x = mapXPoint(point[0]);
    int point_y = mapYPoint(point[1]);

    return isStateValidLocal(Point(point_x, point_y));
}

bool Environment2D::isStateValidLocal(Point point) {
    if (isWhite(point)) return true;
    else return false;
}

int Environment2D::mapXPoint(double x) {
    return (int)((x - lows_[0])/(highs_[0] - lows_[0]) * col);
}

int Environment2D::mapYPoint(double y) {
    return (int)((y - lows_[1])/(highs_[1] - lows_[1]) * (-row) + row);
}

double Environment2D::penetrationDist(pt point, pt& close_point) {
    // return penetration depth of a point in collision. If point not in collision return -1.
    if (isStateValid(point)) {
        return -1;
    } else {
        return 1;
    }
}

bool Environment2D::isWhite(Point pt) {
    Vec3b color = image.at<Vec3b>(pt);
    int b = color[0];
    int g = color[1];
    int r = color[2];
    if (abs(b-255)<10 && abs(g-255)<10 && abs(r-255)<10) { // point is white with some tolerence.
        return true;
    }
    else {
        return false;
    }
}
