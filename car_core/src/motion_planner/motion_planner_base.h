//
// Created by garrus on 14.05.19.
//

#ifndef CAR_CORE_MOTION_PLANNER_BASE_H
#define CAR_CORE_MOTION_PLANNER_BASE_H

#include "motion_planner.h"

class MotionPlannerBase {
public:
    int plan(CarState &state, MotionPlanningTarget &target);
    void get_state_in_frenet_frame(const FrenetFrame &frame, const glm::vec2 &pos, const glm::vec2 &vel, const glm::vec2 &acc, glm::vec3 &s, glm::vec3 &d);
    float calc_baseline_time(const glm::vec3 &s0, const glm::vec3 &s1);
    void calc_lat_trajectory(const glm::vec3 &D0, const glm::vec3 &D1, float t, const range<float> &time_range, Trajectory1D &trajectory);
    void calc_lon_trajectory(const glm::vec3 &S0, const glm::vec3 &S1, float t, const range<float> &time_range, Trajectory1D &trajectory);
    float integrate_jerk(const quintic& coefs, const range<float>& time_range);
};

#endif //CAR_CORE_MOTION_PLANNER_BASE_H
