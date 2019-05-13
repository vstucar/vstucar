//
// Created by garrus on 07.05.19.
//

#ifndef CAR_CORE_MOTION_PLANNER_H
#define CAR_CORE_MOTION_PLANNER_H

#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <vector>

const float ROAD_WIDTH = 3.5*2;         // Ширина дороги - допустимой для езды области
const float D_MIN = -ROAD_WIDTH/2;      // Миимальное значение поперечного положения
const float D_MAX = ROAD_WIDTH/2;       // Максимальное значение поперечного положения
const float D_STEP = 1;                 // Шаг переребора поперечных положений

const float S_DEV = 5;                  // Максимальное отклонение продольного положения вперед/назад от заданного
                                        // si = [s_target - S_DEV, s_target + S_DEV]
const float S_STEP = 3;                 // Шаг перебора продольных положений

const float T_DEV = 0.5;                // Максимальное отклонение времени от примерной оценки
                                        // ti = [(1-T_DEV)*t_estimate, (1+T_DEV)*t_estimate]
const float T_CNT = 4;                  // Количество переборов времени

// Параметры расчета
const float T_CALC_STEP = 0.05;         // Шаг интерполяции по времени

// Параметры ограничений
const float MAX_LON_SPEED = 12;         // Maximum longitudinal speed
const float MIN_LON_SPEED = 0;          // Minimum longitudinal speed (0, to remove strange results)
const float MAX_LON_ACC = 5;            // Maximum longitudinal acceleration
const float MIN_LON_DEACC = -5;         // Minimum longitudinal deacceleration (breaking)
const float MAX_LAT_ACC = 1;            // Maximum lateral acceleration
const float MIN_CURV_RADIUS = 0;        // Minimum curvature radius

// Cost function coefficients
const float K_LAT_J =  1;
const float K_LAT_T =  1;
const float K_LAT_D =  1;
const float K_LON_J =  1;
const float K_LON_T =  1;
const float K_LON_S =  1;
const float K_LON_DS = 1;
const float K_LON    = 1;
const float K_LAT    = 1;

/**
 * Observed car state
 * (instead of ROS msg)
 */
struct CarState
{
    glm::vec2 position;
    glm::vec2 speed;
    float yaw;
};

/**
 * Planning target
 * (instead of ROS msg)
 */
struct MotionPlanningTarget
{
    CarState  state;
    dynarray<glm::vec2> path;
};

/**
 * Store 1D trajectory
 * x(t), x'(t), x''(t)
 *
 *                            |<------ size -------->|
 *                            |                      |
 * |--------------------------|--------------------------------------|------------------------------------------|---------------------------------------------|
 * |x, dx, ddx, size, max_size|x(t0), x(t1), ... x(tn) ... empty ... | x'(t0), x'(t1), ... x'(tn) ... empty ... | x''(t0), x''(t1), ... x''(tn) ... empty ... |
 * |--------------------------|--------------------------------------|------------------------------------------|---------------------------------------------|
 *                            |                                      |
 *                            |<------------- max_size ------------->|
 */
template<class T>
struct Trajectory
{
    Trajectory()
    {
    }

    Trajectory(size_t max_size)
    {
        init(max_size);
    }

    Trajectory(const Trajectory& other)=delete;
    Trajectory(Trajectory&& other)
    {
        move(other);

    }


    Trajectory& operator=(const Trajectory& other) = delete;
    Trajectory& operator=(Trajectory&& other)
    {
        move(other);
        return *this;
    }



    void init(size_t max_size)
    {
        raw = new T[3*max_size];
        this->max_size = max_size;
        x = raw;
        dx = raw + max_size;
        ddx = raw + 2*max_size;
    }

    ~Trajectory()
    {
        if(raw!= nullptr)
        {
            delete[] raw;
            raw = nullptr;
            x = nullptr;
            dx = nullptr;
            ddx = nullptr;
            max_size = 0;
            size = 0;
        }
    }

    std::vector<T> x_to_vec() { return std::vector<T>(x, x+size); }
    std::vector<T> dx_to_vec() { return std::vector<T>(dx, dx+size); }
    std::vector<T> ddx_to_vec() { return std::vector<T>(ddx, ddx+size); }


    float cost = 0;
    bool ok = false;
    T* x = nullptr;
    T* dx = nullptr;
    T* ddx = nullptr;
    size_t size  = 0;

private:

    void move(const Trajectory &other) {
        raw = other.raw;
        x = other.x;
        dx = other.dx;
        ddx = other.ddx;
        max_size = other.max_size;
        size = other.size;

        raw = nullptr;
        x = nullptr;
        dx = nullptr;
        ddx = nullptr;
        max_size = 0;
        size = 0;
    }

    size_t max_size = 0;
    T* raw = nullptr;
};

typedef Trajectory<float> Trajectory1D;
typedef Trajectory<glm::vec2> Trajectory2D;

#endif //CAR_CORE_MOTION_PLANNER_H
