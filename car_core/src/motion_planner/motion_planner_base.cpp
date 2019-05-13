// This file is licensed under MIT license.
// See the LICENSE file in the project root for more information.

#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

#include "dynarray.h"
#include "motion_planner.h"
#include "frenet.h"
#include "quintic.h"
#include "range.h"
#include "matplotlibcpp.h"

using namespace std;
namespace plt=matplotlibcpp;


class MotionPlannerBase
{
public:
    void plan(CarState& state, MotionPlanningTarget& target)
    {
        // Calc current Cartesian position, Cartesian velocity, and orientation (yaw)
        auto pos0 = state.position;
        auto vel0 = state.speed;
        auto yaw0 = state.yaw;

        auto pos1 = target.state.position;
        auto vel1 = target.state.speed;
        auto yaw1 = target.state.yaw;

        // Calc position and velocity in Frenet frame
        FrenetFrame frame(0, target.path[0], target.path[1]);
        glm::vec3 S0, S1, D0, D1;
        get_state_in_frenet_frame(frame, pos0, vel0, glm::vec2(0, 0), S0, D0);
        get_state_in_frenet_frame(frame, pos1, vel1, glm::vec2(0, 0), S1, D1);

        // Estimate maneuver ti_vec
        float t_estimate = calc_baseline_time(S0, S1);

        // Calc bounds of longitudinal and ti_vec variations
        float t_min = (1 - T_DEV) * t_estimate;
        float t_max = (1 + T_DEV) * t_estimate;

        // Create trajectories buffers
        float min_cost = std::numeric_limits<float>::max();
        size_t max_points_per_trajectory = arange<float>(0, t_max, T_CALC_STEP).size();

        auto d_range = arange<float>(D_MIN, D_MAX, D_STEP);
        size_t num_lat_trajectories = d_range.size();

        Trajectory1D* lat_trajectories = new Trajectory1D[num_lat_trajectories];
        for(int i = 0; i<num_lat_trajectories; i++)
            lat_trajectories[i].init(max_points_per_trajectory);

        Trajectory1D* lon_trajectory = new Trajectory1D(max_points_per_trajectory);

        //dynarray<glm::vec2> global_trajectory(max_points_per_trajectory);
        //dynarray<glm::vec2> optimal_trajectory(max_points_per_trajectory);
        Trajectory2D global_trajectory(max_points_per_trajectory);
        Trajectory2D optimal_trajectory(max_points_per_trajectory);

        // TODO: calc trajectories
        for(auto ti: linspace<float>(t_min, t_max, T_CNT))
        {
            std::vector<float> t_vec;
            for(auto t: arange<float>(0, ti.value, T_CALC_STEP))
                t_vec.push_back(t.value);

            for(auto di: d_range)
            {
                calc_lat_trajectory(D0, glm::vec3(di.value, 0, 0), ti.value, &lat_trajectories[ti.index]);

                std::vector<float> d_vec;
                for(int i = 0; i<lat_trajectories[ti.index].size; i++)
                    d_vec.push_back(lat_trajectories[ti.index].ddx[i]);
                plt::plot(t_vec, d_vec);
            }

            /*for (auto si: arange<float>(S1[0] - S_DEV, S1[0] + S_DEV, S_STEP))
            {
                   calc_lon_trajectory(S0, glm::vec3(si.value, S1[1], 0), ti.value, lon_trajectory);

                   for(size_t i = 0; i<num_lat_trajectories; i++)
                   {
                       float cost = K_LAT * lat_trajectories[i].cost + K_LON + lon_trajectory->cost;
                       FrenetFrame::path_to_global(*lon_trajectory, lat_trajectories[i], target.path, global_trajectory);
                       global_trajectory.cost = cost;

                       if(global_trajectory.cost < optimal_trajectory.cost)
                           std::swap(global_trajectory, optimal_trajectory);
                   }
            }*/
        }


        delete[] lat_trajectories;
        delete lon_trajectory;
        plt::show();

    }

    void get_state_in_frenet_frame(const FrenetFrame& frame, const glm::vec2& pos, const glm::vec2& vel, const glm::vec2& acc,
                                   glm::vec3& s, glm::vec3& d)
    {
        auto pos_f = frame.point_to(pos);
        auto vel_f = frame.vector_to(vel);
        auto acc_f = frame.vector_to(acc);
        s = glm::vec3(pos_f.x, vel_f.x, acc_f.x);
        d = glm::vec3(pos_f.y, vel_f.y, acc_f.y);
    }

    float calc_baseline_time(const glm::vec3& s0, const glm::vec3& s1)
    {
        return (2 * (s1[0] - s0[0])) / (s0[1] + s1[1]);
    }

    void calc_lat_trajectory(const glm::vec3& D0, const glm::vec3& D1, float t, Trajectory1D* trajectory)
    {
        cout << D0[0] << " " << D0[1] << " " << D0[2] << endl;
        cout << D1[0] << " " << D1[1] << " " << D1[2] << endl;
        cout << endl;

        quintic coefs(D0, D1, t);
        auto range = arange<float>(0, t, T_CALC_STEP);

        for(auto ti: range)
            trajectory->x[ti.index] = coefs.x(ti.value);

        for(auto ti: range)
            trajectory->dx[ti.index] = coefs.dx(ti.value);

        bool ddx_constraints_ok = true;
        for(auto ti: range)
        {
            trajectory->ddx[ti.index] = coefs.ddx(ti.value);
            ddx_constraints_ok &= fabs(trajectory->ddx[ti.index]) <= MAX_LAT_ACC;
        }

        trajectory->ok = ddx_constraints_ok;
        trajectory->cost = 0;
        trajectory->size = range.size();
    }

    void calc_lon_trajectory(const glm::vec3& S0, const glm::vec3& S1, float t, Trajectory1D* trajectory)
    {
        quintic coefs(S0, S1, t);
        auto range = arange<float>(0, t, T_CALC_STEP);

        for(auto ti: range)
            trajectory->x[ti.index] = coefs.x(ti.value);

        bool dx_constraints_ok = true;
        for(auto ti: range)
        {
            trajectory->dx[ti.index] = coefs.dx(ti.value);
            dx_constraints_ok &= trajectory->dx[ti.index] <= MAX_LON_SPEED && trajectory->dx[ti.index] >= MIN_LON_SPEED;
        }

        bool ddx_constraints_ok = true;
        for(auto ti: range)
        {
            trajectory->ddx[ti.index] = coefs.dx(ti.value);
            ddx_constraints_ok &= fabs(trajectory->ddx[ti.index]) <= MAX_LAT_ACC;
        }

        trajectory->ok = ddx_constraints_ok && dx_constraints_ok;
        trajectory->cost = 0;
        trajectory->size = range.size();
    }

private:

};

int main()
{
    const int N_POINTS = 10;
    const float SPEED = 8;
    const float TARGET_X = 10;
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
    planner.plan(cur_state, target);

    return 0;
}
