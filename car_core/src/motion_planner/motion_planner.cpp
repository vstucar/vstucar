#include "motion_planner.h"
#include "motion_planner_base.h"
#include <omp.h>
#include <iostream>
#include <fstream>

int main()
{
    ///// Prepare data //////
    const int N_POINTS = 10;
    const float SPEED = 15;
    const float TARGET_X = 60;
    const float TARGET_SPEED = 0;

    CarState cur_state{{0, 0}, {SPEED, 0}, 0};
    MotionPlanningTarget target{{{TARGET_X, 0}, {TARGET_SPEED, 0}, 0}, dynarray<glm::vec2>(N_POINTS)};

    float dx = TARGET_X / N_POINTS;
    float x = 0;
    for(int i = 0; i<N_POINTS; i++)
    {
        target.path[i].x = x;
        x+=dx;
    }

    MotionPlannerBase planner;

    /////////// Run /////////
    const size_t iters = 10;

    std::ofstream f("serial.txt");
    std::cout << "Serial\n";
    planner.plan(cur_state, target); // warm-up

    for(size_t i = 0; i<iters; i++)
    {
        double start = omp_get_wtime();
        int count = planner.plan(cur_state, target); // warm-up
        double duration = omp_get_wtime() - start;

        f<< duration << " ";
        std::cout << duration << std::endl;
        std::cout << "tr count: " << count << std::endl;
    }

    f.close();

    return 0;
}