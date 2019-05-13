//
// Created by garrus on 07.05.19.
//

#ifndef CAR_CORE_MOTION_PLANNER_H
#define CAR_CORE_MOTION_PLANNER_H

#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <vector>

#include "dynarray.h"
#include "quintic.h"
#include "range.h"
#include "trajectory.h"
#include "frenet.h"

const float ROAD_WIDTH = 3.5*2;         // Ширина дороги - допустимой для езды области
const float D_MIN = -ROAD_WIDTH/2;      // Миимальное значение поперечного положения
const float D_MAX = ROAD_WIDTH/2;       // Максимальное значение поперечного положения
const float D_STEP = 0.25;              // Шаг переребора поперечных положений

const float S_DEV = 15;                 // Максимальное отклонение продольного положения вперед/назад от заданного
                                        // si = [s_target - S_DEV, s_target + S_DEV]
const float S_STEP = 1;                 // Шаг перебора продольных положений

const float T_DEV = 0.7;                // Максимальное отклонение времени от примерной оценки
                                        // ti = [(1-T_DEV)*t_estimate, (1+T_DEV)*t_estimate]
const float T_CNT = 10;                 // Количество переборов времени

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

#endif //CAR_CORE_MOTION_PLANNER_H
