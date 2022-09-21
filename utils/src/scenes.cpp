#include "scenes.h"


Environment* get_scene_env(int scene)
{
    pt start;
    pt goal;
    Environment* env;
    pt lows;  // joint limits 
    pt highs;
    double delta = 0;  // the boundaries virtual obstacle region's size and the virtual cfree region's size

    switch(scene) {
        case 1:
            // 2D
            start = {0, 1.2};
            goal = {0, -1.0};
            lows = {-4, -4.0};
            highs = {4, 4};
            env = new Environment2D("../resource/exist2.png", lows, highs, start, goal);
            break;
        case 2: {
            // zyzyzyz 7dof
            start = {-2.47, 2.5093, 0.5, -2.54409, -0.1, 1.72318, -0.30};
            goal = {-1.09912, 2.5093, 0.5, -2.54409, -0.1, 1.72318, -0.30};
            std::vector<std::string> joints = {"s0", "s1", "s2", "e", "w0", "w1", "w2"};
            delta = 0.2;
            lows =  {-M_PI, -2.8, -M_PI/2, -2.8, -M_PI/2, -2.8, -M_PI/2};
            highs = {0.0,   2.8, M_PI/2, 2.8, M_PI/2, 2.8, M_PI/2};

            env = new HighDofEnvironment(start, goal, joints, "7dofexistmedium", "../resource/7dof_exist_medium/libscene.so", lows, highs, delta);
            break;
        }
        case 3: {
            // 6dof universal arm
            start = {-1.6, 1.13, 2.32, 2.08, 3.32, 0.68};
            goal = {-0.07, -0.56, 1.52, -0.47, 4.69, 0};
            std::vector<std::string> joints = {"s0", "s1", "e", "w0", "w1", "w2"};
            delta = 0.2;
            lows =  {-M_PI, -2.8, -M_PI, -2.8, 0, -M_PI/2};
            highs = {M_PI,   2.8, M_PI, 2.8, 2*M_PI, M_PI/2};

            env = new HighDofEnvironment(start, goal, joints, "6doftabletop", "../resource/6dof_tabletop/libscene.so", lows, highs, delta);
            break;
        }

        case 4: {
            // 6dof, 3 2d robots
            double w = 0.08;

            pvec squares {
                {0, 0, 0.5, 4.5},
                {0.5, 4, 4.5, 4.5},
                {0.5+w, 1.5, 3.5, 4.0-w},
                {2.25, 0, 3.5, 1.5}
            };

            start = {0.8, 0.5, 1.2, 0.5, 1.6, 0.5};
            goal = {0.5 + w/2, 0.7, 0.5 + w/2, 4.0 - w/2, 4.0, 4.0 - w/2};

            lows =  {0, 0, 0, 0, 0, 0};
            highs = {4.5, 4.5, 4.5, 4.5, 4.5, 4.5};

            delta = 0.1;
            env = new MultiRobotEnv(lows, highs, start, goal, squares, delta);
            break;
        }
        case 5: {
            // 6 dof twistycool
            start = {0, 1.5, 0, 0, 0, 0};
            goal = {0, -1.5, 0, 0, 0, 0};
            std::vector<std::string> joints = {"p1", "p2", "p3", "r1", "r2", "r3"};
            lows =  {-3, -3, 0, -M_PI, -M_PI, -M_PI};
            highs = {3, 3, 3, M_PI, M_PI, M_PI};

            env = new HighDofEnvironment(start, goal, joints, "twistycool", "../resource/twistycool_own/libscene.so", lows, highs, delta);
            break;
        }
    }

    return env;
    
}