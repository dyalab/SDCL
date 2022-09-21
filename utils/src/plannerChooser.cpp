#include "plannerChooser.h"

name_code get_code(std::string const& name) {
    if (name == "SDCL") return SDCL;
    if (name == "KPIECE1") return KPIECE1;
    if (name == "LBKPIECE1") return LBKPIECE1;
    if (name == "BKPIECE1") return BKPIECE1;
    if (name == "BFMT") return BFMT;
    if (name == "RRT") return RRT;
    if (name == "LazyRRT") return LazyRRT;
    if (name == "TRRT") return TRRT;
    if (name == "LBTRRT") return LBTRRT;
    if (name == "RRTConnect") return RRTConnect;
    if (name == "BiTRRT") return BiTRRT;
    if (name == "EST") return EST;
    if (name == "SBL") return SBL;
    if (name == "PRM") return PRM;
    if (name == "LazyPRM") return LazyPRM;
}

ompl::base::PlannerPtr plannerChooser(const ompl::base::SpaceInformationPtr &si, std::string planner_name, bool use_training, bool use_Gaussian) {
    switch(get_code(planner_name)) {
        case SDCL: {
            return std::make_shared<SDCL>(si);
        }
        case KPIECE1: {
            return std::make_shared<og::KPIECE1>(si);
        }
        case LBKPIECE1: {
            return std::make_shared<og::LBKPIECE1>(si);
        }
        case BKPIECE1: {
            return std::make_shared<og::BKPIECE1>(si);
        }
        case BFMT: {
            return std::make_shared<og::BFMT>(si);
        }
        case RRT: {
            return std::make_shared<og::RRT>(si);
        }
        case LazyRRT: {
            return std::make_shared<og::LazyRRT>(si);
        }
        case TRRT: {
            return std::make_shared<og::TRRT>(si);
        }
        case LBTRRT: {
            return std::make_shared<og::LBTRRT>(si);
        }
        case RRTConnect: {
            return std::make_shared<og::RRTConnect>(si);
        }
        case BiTRRT: {
            return std::make_shared<og::BiTRRT>(si);
        }
        case EST: {
            return std::make_shared<og::EST>(si);
        }
        case SBL: {
            return std::make_shared<og::SBL>(si);
        }
        case PRM: {
            return std::make_shared<og::PRM>(si);
        }
        case LazyPRM: {
            return std::make_shared<og::LazyPRM>(si);
        }
    }

}