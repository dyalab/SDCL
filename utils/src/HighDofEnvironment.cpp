#include <environment.h>
  
HighDofEnvironment::HighDofEnvironment(pt start, pt goal, std::vector<std::string>& joints, 
                                       const char* scene_name, const char* path, pt lows, pt highs, double delta) {
    start_ = start;
    goal_ = goal;
    delta_ = delta; // the boundaries virtual obstacle region's size and the virtual cfree region's size
    highs_ = highs; // original joints limits.
    lows_ = lows;
    N_ = start.size();
    setup_scene(joints, scene_name, path);
}

void HighDofEnvironment::setup_scene(std::vector<std::string>& joints, const char* scene_name, const char* path) {
    sg_ = aa_rx_sg_create();
    aa_rx_dl_sg_at(path, scene_name, sg_, "");
    aa_rx_sg_init(sg_);

    // fk
    fk_ = aa_rx_fk_malloc(sg_);

    // collision
    aa_rx_sg_cl_init(sg_);
    cl_ = aa_rx_cl_create(sg_);

    // collision set
    cl_set_ = aa_rx_cl_set_create(sg_);

    // collision dist 
    cl_dist_ = aa_rx_cl_dist_create(cl_);

    // initiate config
    config_ = new aa_dvec[1];
    config_->len = aa_rx_sg_config_count(sg_);
    config_->data = new double[config_->len];
    config_->inc = 1;

    // get config id
    ids_ = new aa_rx_config_id[N_];
    for (int i = 0; i < N_; i++) {
        ids_[i] = aa_rx_sg_config_id(sg_, (joints[i]).c_str());
        // std::cout << ids_[i] << std::endl;
    }

    // pt test = {12.6131875, 10.204525, 3.09005375, 0.25954625, -0.15721625, -0.2509875};

    for (int i = 0; i < config_->len; i++) {
        config_->data[ids_[i]] = start_[i];
        // config_->data[ids_[i]] = test[i];
    }


    // set allow set
    aa_rx_fk_all(fk_, config_);
    aa_rx_cl_set_clear(cl_set_);
    int res = aa_rx_cl_check(cl_, aa_rx_fk_cnt(fk_), aa_rx_fk_data(fk_), aa_rx_fk_ld(fk_), cl_set_);
    // std::cout << res << std::endl;
    aa_rx_cl_allow_set(cl_, cl_set_);
    

    std::cout << "Scene setup done!" << std::endl;
    // saved2dPoints_.open("cfree_points.csv");
}

HighDofEnvironment::~HighDofEnvironment() {
    aa_rx_cl_dist_destroy(cl_dist_);
    aa_rx_cl_set_destroy(cl_set_);
    aa_rx_cl_destroy(cl_);
    aa_rx_fk_destroy(fk_);
    aa_rx_sg_destroy(sg_);
    delete [] ids_;
    delete [] config_->data;
    delete [] config_;
    // saved2dPoints_.close();
}

bool HighDofEnvironment::outOfLimits(pt point) {
    // whether point is out of joint limits
    for (int i = 0; i < N_; i++) {
        if (point[i] < lows_[i] || point[i] > highs_[i]) {
            return true;
        }
    }
    return false;
}

bool HighDofEnvironment::outOfLimitsCollision(pt point) {
    for (int i = 0; i < N_; i++) {
        if (point[i] < lows_[i] - delta_ || point[i] > highs_[i] + delta_) {
            return true;
        }
    }
    return false;
}

bool HighDofEnvironment::isStateValid(pt point) {
    if (delta_ > 0) {
        if (outOfLimitsCollision(point)) {
            return true;
        }
        if (outOfLimits(point)) {
            return false;
        }
    }

    int res = isStateValid_(point);
    if (res == 0) {
        return true;
    } else {
        return false;
    }
}

int HighDofEnvironment::isStateValid_(pt point) {
    aa_dvec* config = new aa_dvec[1];
    config->len = N_;
    config->data = new double[N_];
    config->inc = 1;

    for (int i = 0; i < config->len; i++) {
        config->data[ids_[i]] = point[i];
    }

    aa_rx_fk* fk = aa_rx_fk_malloc(sg_);
    aa_rx_cl* cl = aa_rx_cl_create(sg_);
    aa_rx_cl_allow_set(cl, cl_set_);
    aa_rx_fk_all(fk, config);
    int res = aa_rx_cl_check(cl, aa_rx_fk_cnt(fk), aa_rx_fk_data(fk), aa_rx_fk_ld(fk), NULL);
    // std::cout << " res is" << res << std::endl;
    delete [] config->data;
    delete [] config;
    aa_rx_fk_destroy(fk);
    aa_rx_cl_destroy(cl);
    return res;
}

double HighDofEnvironment::penetrationDist(pt point, pt& close_point) {
    return 0;
}