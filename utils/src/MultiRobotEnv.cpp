#include "environment.h"

// multi 2d robot env
MultiRobotEnv::MultiRobotEnv(pt lows, pt highs, pt start, pt goal, pvec squares, double delta) {
    // squares = [[lower_left_x, lower_left_y, upper_right_x, upper_right_y], []...]
    // n is the number of robots. 
    highs_ = highs; // joints limits.
    lows_ = lows;
    goal_ = goal;
    start_ = start;
    N_ = start.size();
    delta_= 0;
    squares_ = squares;
    n_robots_ = N_/2;
    robots_ = pt(N_, 0);
    delta_ = delta;
}

bool MultiRobotEnv::isStateValid(pt point) {
    // point = [x1, y1, x2, y2, x3, y3]
    if (delta_ > 0) {
        if (outOfLimitsCollision(point)) {
            return true;
        }
        if (outOfLimits(point)) {
            return false;
        }
    }

    for (int i = 0; i < N_; i++) {
        robots_[i] = point[i];
    }

    // robots in squares
    for (int r = 0; r < n_robots_; r++) {
        for (int s = 0; s < squares_.size(); s++) {
            if (robotInSquare(r, s)) return false;
        }
    }
    
    // maintain line of sights
    for (int r1 = 0; r1 < n_robots_ - 1; r1++) {
        for (int s = 0; s < squares_.size(); s++) {
            if (lineIntersectSquare(r1, r1+1, s)) return false;
        }
    }
    
    return true;
}

bool MultiRobotEnv::robotInSquare(int r, int s) {
    // std::cout << robots_[2*r] << ' ' << robots_[2*r+1]  << " " << squares_[s][0] << " " << squares_[s][2] << " " << squares_[s][1] << " " << squares_[s][3] << std::endl;
    if (squares_[s][0] < robots_[2*r] && robots_[2*r] < squares_[s][2] && 
        squares_[s][1] < robots_[2*r+1] && robots_[2*r+1] < squares_[s][3]) return true;
    else return false;
}
bool MultiRobotEnv::lineIntersectSquare(int r1, int r2, int s) {
    pt p1 = {robots_[r1*2], robots_[r1*2 + 1]};
    pt p2 = {robots_[r2*2], robots_[r2*2 + 1]};

    if (lineIntersectVerticalLine(p1, p2, squares_[s][0], squares_[s][1], squares_[s][3]) || 
        lineIntersectVerticalLine(p1, p2, squares_[s][2], squares_[s][1], squares_[s][3]) ||
        lineIntersectHorizontalLine(p1, p2, squares_[s][1], squares_[s][0], squares_[s][2]) ||
        lineIntersectHorizontalLine(p1, p2, squares_[s][3], squares_[s][0], squares_[s][2])) return true;
    else return false;

}
bool MultiRobotEnv::lineIntersectVerticalLine(pt p1, pt p2, double x, double y1, double y2) {
    if (x < std::min(p1[0], p2[0]) || x > std::max(p1[0], p2[0])) return false;

    double y = (p2[1] - p1[1]) * (x - p1[0]) / (p2[0] - p1[0]) + p1[1];

    if (y1 < y && y < y2) return true;
    else return false;
}
bool MultiRobotEnv::lineIntersectHorizontalLine(pt p1, pt p2, double y, double x1, double x2) {
    if (y < std::min(p1[1], p2[1]) || y > std::max(p1[1], p2[1])) return false;

    double x = (p2[0] - p1[0]) * (y - p1[1]) / (p2[1] - p1[1]) + p1[0];

    if (x1 < x && x < x2) return true;
    else return false;
}

bool MultiRobotEnv::outOfLimits(pt point) {
    // whether point is out of joint limits
    for (int i = 0; i < N_; i++) {
        if (point[i] < lows_[i] || point[i] > highs_[i]) {
            return true;
        }
    }
    return false;
}

bool MultiRobotEnv::outOfLimitsCollision(pt point) {
    for (int i = 0; i < N_; i++) {
        if (point[i] < lows_[i] - delta_ || point[i] > highs_[i] + delta_) {
            return true;
        }
    }
    return false;
}

double MultiRobotEnv::penetrationDist(pt point, pt& close_point) {
    // return penetration depth of a point in collision. If point not in collision return -1.
    if (isStateValid(point)) {
        return -1;
    } else {
        return 1;
    }
}

