// This file is licensed under MIT license.
// See the LICENSE file in the project root for more information.

#include <iostream>
#include "motion_planner_base.h"
#include "matplotlibcpp.h"

using namespace std;
namespace plt=matplotlibcpp;


int MotionPlannerBase::plan(CarState &state, MotionPlanningTarget &target) {
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
    float t_min = (1.0f - T_DEV) * t_estimate;
    float t_max = (1.0f + T_DEV) * t_estimate;

    /*cout << "t_estimate: " << t_estimate << endl;
    cout << "t_min:      " << t_min << endl;
    cout << "t_max:      " << t_max << endl;*/

    // Create trajectories buffers
    float min_cost = std::numeric_limits<float>::max();
    size_t max_points_per_trajectory = arange<float>(0, t_max, T_CALC_STEP).size();

    auto d_range = arange<float>(D_MIN, D_MAX, D_STEP);
    size_t num_lat_trajectories = d_range.size();

    Trajectory1D *lat_trajectories = new Trajectory1D[num_lat_trajectories];
    for (int i = 0; i < num_lat_trajectories; i++)
        lat_trajectories[i].init(max_points_per_trajectory);

    Trajectory1D lon_trajectory(max_points_per_trajectory);

    //dynarray<glm::vec2> global_trajectory(max_points_per_trajectory);
    //dynarray<glm::vec2> optimal_trajectory(max_points_per_trajectory);
    Trajectory2D global_trajectory(max_points_per_trajectory);
    Trajectory2D optimal_trajectory(max_points_per_trajectory);

    int count = 0;

    for (auto ti: linspace<float>(t_min, t_max, T_CNT)) {
        auto t_range = arange<float>(0, ti.value, T_CALC_STEP);
        //auto t_vec = t_range.to_vector();

        //cout << "Generate lat\n";
        for (auto di: d_range) {
            calc_lat_trajectory(D0, glm::vec3(di.value, 0, 0), ti.value, t_range, lat_trajectories[di.index]);
            //auto d_vec = lat_trajectories[di.index].x_to_vec();
            //plt::plot(t_vec, d_vec);
        }

        //cout << "Lon + lat\n";
        //cout << S1[0] - S_DEV << " " << S1[0] + S_DEV << " " << S_STEP << endl;
        for (auto si: arange<float>(S1[0] - S_DEV, S1[0] + S_DEV, S_STEP)) {
            calc_lon_trajectory(S0, glm::vec3(si.value, S1[1], 0), ti.value, t_range, lon_trajectory);
            auto s_vec = lon_trajectory.x_to_vec();
            //plt::plot(t_vec, s_vec);

            for (size_t i = 0; i < num_lat_trajectories; i++) {
                float cost = K_LAT * lat_trajectories[i].cost + K_LON + lon_trajectory.cost;
                FrenetFrame::path_to_global(lon_trajectory, lat_trajectories[i], target.path, global_trajectory);
                global_trajectory.cost = cost;
                count++;

                //auto d_vec = lat_trajectories[i].x_to_vec();
                //plt::plot(s_vec, d_vec);

                if (global_trajectory.cost < optimal_trajectory.cost)
                    std::swap(global_trajectory, optimal_trajectory);
            }
        }
    }


    delete[] lat_trajectories;

    return count;

}


void MotionPlannerBase::get_state_in_frenet_frame(const FrenetFrame &frame, const glm::vec2 &pos, const glm::vec2 &vel,
                               const glm::vec2 &acc,
                               glm::vec3 &s, glm::vec3 &d) {
    auto pos_f = frame.point_to(pos);
    auto vel_f = frame.vector_to(vel);
    auto acc_f = frame.vector_to(acc);
    s = glm::vec3(pos_f.x, vel_f.x, acc_f.x);
    d = glm::vec3(pos_f.y, vel_f.y, acc_f.y);
}

float MotionPlannerBase::calc_baseline_time(const glm::vec3 &s0, const glm::vec3 &s1) {
    return (2 * (s1[0] - s0[0])) / (s0[1] + s1[1]);
}

void MotionPlannerBase::calc_lat_trajectory(const glm::vec3 &D0, const glm::vec3 &D1, float t, const range<float> &time_range,
                         Trajectory1D &trajectory) {
    quintic coefs(D0, D1, t);

    /*cout << "lat\n";
    cout << "DO: " << D0[0] << " " << D0[1] << " " << D0[2] << endl;
    cout << "D1: " << D1[0] << " " << D1[1] << " " << D1[2] << endl;
    cout << "T:  " << t << ", N: " << time_range.size() << endl;
    cout << endl;*/

    for (auto ti: time_range)
        trajectory.x[ti.index] = coefs.x(ti.value);

    for (auto ti: time_range)
        trajectory.dx[ti.index] = coefs.dx(ti.value);

    bool ddx_constraints_ok = true;
    for (auto ti: time_range) {
        trajectory.ddx[ti.index] = coefs.ddx(ti.value);
        ddx_constraints_ok &= fabs(trajectory.ddx[ti.index]) <= MAX_LAT_ACC;
    }

    trajectory.cost = K_LAT_J * integrate_jerk(coefs, time_range) +
                      K_LAT_T * t +
                      K_LAT_D * powf(trajectory.x[trajectory.size-1] - D1[0], 2);

    trajectory.ok = ddx_constraints_ok;
    trajectory.size = time_range.size();
}

void MotionPlannerBase::calc_lon_trajectory(const glm::vec3 &S0, const glm::vec3 &S1, float t, const range<float> &time_range,
                         Trajectory1D &trajectory) {
    quintic coefs(S0, S1, t);

    /*cout << "lon\n";
    cout << "SO: " << S0[0] << " " << S0[1] << " " << S0[2] << endl;
    cout << "S1: " << S1[0] << " " << S1[1] << " " << S1[2] << endl;
    cout << "T:  " << t << ", N: " << time_range.size() << endl;
    cout << endl;*/

    for (auto ti: time_range)
        trajectory.x[ti.index] = coefs.x(ti.value);

    bool dx_constraints_ok = true;
    for (auto ti: time_range) {
        trajectory.dx[ti.index] = coefs.dx(ti.value);
        dx_constraints_ok &= trajectory.dx[ti.index] <= MAX_LON_SPEED && trajectory.dx[ti.index] >= MIN_LON_SPEED;
    }

    bool ddx_constraints_ok = true;
    for (auto ti: time_range) {
        trajectory.ddx[ti.index] = coefs.dx(ti.value);
        ddx_constraints_ok &= fabs(trajectory.ddx[ti.index]) <= MAX_LAT_ACC;
    }

    trajectory.cost = K_LON_J * integrate_jerk(coefs, time_range) +
                      K_LON_T * t +
                      K_LON_S * powf(trajectory.x[trajectory.size-1] - S1[0], 2) +
                      K_LON_DS * powf(trajectory.dx[trajectory.size-1] - S1[1], 2);

    trajectory.ok = ddx_constraints_ok && dx_constraints_ok;
    trajectory.size = time_range.size();
}

float MotionPlannerBase::integrate_jerk(const quintic& coefs, const range<float>& time_range)
{
    float jerk_int = 0;
    for (auto ti: time_range)
        jerk_int += powf(coefs.jerk(ti.value), 2);
    return jerk_int * T_CALC_STEP;
}

