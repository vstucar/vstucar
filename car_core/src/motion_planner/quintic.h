// This file is licensed under MIT license.
// See the LICENSE file in the project root for more information.

#ifndef CAR_CORE_QUINTIC_H
#define CAR_CORE_QUINTIC_H

#include <array>
#include <glm/glm.hpp>
#include "range.h"

class quintic
{
public:
    quintic(const glm::vec3 s0, const glm::vec3& s1, float T)
    {
        coefs[0] = (T*T*(-s0[2] + s1[2]) - 6*T*(s0[1] + s1[1]) - 12*s0[0] + 12*s1[0])/(2*pow(T,5));
        coefs[1] = (T*T*(3*s0[2] - 2*s1[2])/2 + T*(8*s0[1] + 7*s1[1]) + 15*s0[0] - 15*s1[0])/powf(T,4);
        coefs[2] = (T*T*(-3*s0[2] + s1[2]) - 4*T*(3*s0[1] + 2*s1[1]) - 20*s0[0] + 20*s1[0])/(2*powf(T,3));
        coefs[3] = s0[2]/2;
        coefs[4] = s0[1];
        coefs[5] = s0[0];
    }

    inline __attribute__((always_inline))
    float x(float t) const
    {
        return coefs[0]*powf(t, 5) + coefs[1]*powf(t, 4) + coefs[2]*powf(t, 3) + coefs[3]*powf(t, 2) + coefs[4]*t + coefs[5];
    }

    inline __attribute__((always_inline))
    float dx(float t) const
    {
        return 5*coefs[0]*powf(t, 4) + 4*coefs[1]*powf(t, 3) + 3*coefs[2]*powf(t, 2) + 2*coefs[3]*t + coefs[4];
    }


    inline __attribute__((always_inline))
    float ddx(float t) const
    {
        return 20*coefs[0]*powf(t, 3) + 12*coefs[1]*powf(t, 2) + 6*coefs[2]*t + 2*coefs[3];
    }


    inline __attribute__((always_inline))
    float jerk(float t) const
    {
        return 60*coefs[0]*powf(t, 2) + 24 * coefs[1]*t + 6*coefs[2];
    }
    
private:
    std::array<float, 6> coefs;

    /**
     * Interpolates x(t), x'(t), x''(t) for given t array
     * Interpolated values stored in plane array. Every function should have less
     * or equal max_trajectory_size elements
     *
     *
     * @param coefs Quntic polinomial coefficients
     * @param t Argument values
     * @param trajectory Calculated trajectories
     */
    /*void interpolate(const std::array<float, 6>& coefs, range<float>& t,  Trajectory1D& trajectory)
    {
        // x(t)
        size_t i = 0;
        for(float ti: t)
        {
            trajectory.x[i] = coefs[0]*powf(ti, 5) + coefs[1]*powf(ti, 4) + coefs[2]*powf(ti, 3) + coefs[3]*powf(ti, 2) + coefs[3]*ti + coefs[4];
            i++;
        }

        //x'(t)
        i = 0;
        for(float ti: t)
        {
            trajectory.dx[i] = 5*coefs[0]*powf(ti, 4) + 4*coefs[1]*powf(ti, 3) + 3*coefs[2]*powf(ti, 2) + 2*coefs[3]*ti + coefs[3];
            i++;
        }

        //x''(t)
        i = 0;
        for(float ti: t)
        {
            trajectory.ddx[i] = 20*coefs[0]*powf(ti, 3) + 12*coefs[1]*powf(ti, 2) + 6*coefs[2]*ti + 2*coefs[3];
            i++;
        }
        trajectory.size = i;
    }*/

};


#endif //CAR_CORE_QUINTIC_H
