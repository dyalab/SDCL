#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <opencv2/opencv.hpp>
#include <map>
#include <tuple>
#include <mutex>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <iostream>
#include <set>
#include <math.h>
#include <mutex>
#include <chrono>
#include <filesystem>
#include <atomic>
#include <fstream>

#include <amino.h>
#include <amino/rx/rxtype.h>
#include <amino/rx/scene_collision.h>
#include <amino/rx/scenegraph.h>
#include <amino/rx/scene_plugin.h>
#include <amino/rx/scene_fk.h>
#include <amino/rx/scene_win.h>
#include <amino/mat.h>

typedef std::vector<std::vector<double>> pvec;
typedef std::vector<double> pt;
typedef std::map<pt, std::vector<double> > dist_hash;
typedef std::map<pt, std::vector<double*> > point_hash;


class Environment {
public:
    Environment(){};
    virtual ~Environment(){};  // need to define this virtual so children destructor are called. 
    virtual bool isStateValid(pt point)=0;
    virtual double penetrationDist(pt point, pt& close_point)=0;
    pt get_start() {return start_;}
    pt get_goal() {return goal_;};
    pt get_highs() {return highs_;};
    pt get_lows() {return lows_;};
    int get_dim() {return N_;};
    double get_delta() {return delta_;};
    void set_lows(pt& lows) {lows_ = lows;};
    void set_highs(pt& highs) {highs_ = highs;};
protected:
    pt goal_;
    pt start_;
    pt highs_;
    pt lows_;
    int N_;
    double delta_;
};

class HyperBallEnvironment: public Environment {
public:
    HyperBallEnvironment(pt lows, pt highs, pt start, pt goal, int N);
    ~HyperBallEnvironment();
    bool isStateValid(pt point);
    double penetrationDist(pt point, pt&close_point);
private:
    double inner_radius;
    double outter_radius;
};

class Environment2D: public Environment {
public: 
    Environment2D(const std::string path, pt lows, pt highs, pt start, pt goal);
    ~Environment2D();
    bool isStateValid(pt point);
    bool isStateValidLocal(cv::Point point);
    double penetrationDist(pt point, pt& close_point);
    int mapXPoint(double x);
    int mapYPoint(double y);
    cv::Mat image;
    int row;
    int col;
private:
    int mapPoint(double x, double l1, double h1, double l2, double h2);
    bool isWhite(cv::Point pt);
};

class HighDofEnvironment: public Environment {
public:
    HighDofEnvironment(pt start, pt goal, std::vector<std::string>& joints, const char* scene_name, const char* path, pt lows, pt highs,
                       double delta = 0.0);
    ~HighDofEnvironment();
    bool isStateValid(pt point);
    double penetrationDist(pt point, pt& close_point);
    // double penetrationDisk(pt point, pt close_point, double* hyper_plane_params);
    aa_rx_sg* sg_;
    aa_rx_fk* fk_;
    aa_rx_cl* cl_;
    aa_rx_cl_set* cl_set_;
    aa_rx_cl_dist* cl_dist_;
    aa_rx_config_id* ids_;
    aa_dvec* config_;
    std::mutex scene_mutex;
    std::vector<std::tuple<std::string, std::string>> linkpair_list;
    
private:
    void setup_scene(std::vector<std::string>& joints, const char* scene_name, const char* path);
    bool outOfLimits(pt point);
    bool outOfLimitsCollision(pt point);
    int isStateValid_(pt point);  // is valid without the joints limits
    // std::ofstream saved2dPoints_;
};

class MultiRobotEnv: public Environment {
public: 
    MultiRobotEnv(pt lows, pt highs, pt start, pt goal, pvec squares, double delta);
    ~MultiRobotEnv(){};
    bool isStateValid(pt point);
    bool robotInSquare(int r, int s);
    bool lineIntersectSquare(int r1, int r2, int s);
    bool lineIntersectVerticalLine(pt p1, pt p2, double x, double y1, double y2);
    bool lineIntersectHorizontalLine(pt p1, pt p2, double y, double x1, double x2);
    bool outOfLimits(pt point);
    bool outOfLimitsCollision(pt point);
    double penetrationDist(pt point, pt& close_point);
private:
    std::vector<pt> squares_;  // squares = [[lower_left_x, lower_left_y, upper_right_x, upper_right_y], []...]
    pt robots_;  // robot positions [x1, y1, x2, y2, x3, y3]
    int n_robots_;  // number of robots. 
};


class PosScaleEnv: public Environment {
public:
    PosScaleEnv(pt start, pt goal, std::vector<std::string>& joints, 
                const char* scene_name, const char* path, pt lows, pt highs, 
                double pos_scale,
                double delta = 0.0);
    ~PosScaleEnv();
    bool isStateValid(pt point);
    double penetrationDist(pt point, pt& close_point);
    // double penetrationDisk(pt point, pt close_point, double* hyper_plane_params);
    aa_rx_sg* sg_;
    aa_rx_fk* fk_;
    aa_rx_cl* cl_;
    aa_rx_cl_set* cl_set_;
    aa_rx_cl_dist* cl_dist_;
    aa_rx_config_id* ids_;
    aa_dvec* config_;
    
private:
    void setup_scene(std::vector<std::string>& joints, const char* scene_name, const char* path);
    bool outOfLimits(pt point);
    bool outOfLimitsCollision(pt point);
    int isStateValid_(pt point);  // is valid without the joints limits
    pt scale_point(pt point);
    double pos_scale_;
    // std::ofstream saved2dPoints_;
};

#endif