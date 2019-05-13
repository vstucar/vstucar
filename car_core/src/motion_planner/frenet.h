//
// Created by garrus on 07.05.19.
//

#ifndef CAR_CORE_FRENET_H
#define CAR_CORE_FRENET_H

#include <glm/glm.hpp>

/**
 * @brief Represents the Frenet Frame
 */
class FrenetFrame
{
public:

    /**
     * @brief  Create Frenet Frame with two close points on discrete curve. Frenet Frame origin will be in point p1
     * @param s0
     * @param p1
     * @param p2
     */
    FrenetFrame(const float& s0, const glm::vec2& p1, const glm::vec2& p2)
    {
        // local frame axis in parent frame
        tr = p2 - p1;
        glm::normalize(tr);
        nr = glm::vec2(glm::cross(glm::vec3(0, 0, 1), glm::vec3(tr, 0)));

        Rwf[0] = tr;
        Rwf[1] = nr;

        Rfw = glm::transpose(Rwf);
        Pwf = p1;
        S0 = glm::vec2(s0, 0);
    }

    /**
     * @brief Transofrms point in parent frame to the local frame
     * @param point
     * @return
     */
    glm::vec2 point_to(const glm::vec2& point) const
    {
        return Rfw * (point - Pwf) + S0;
    }

    /**
     * @brief Transofrms point in local frame to the parent frame
     * @param point
     * @return
     */
    glm::vec2 point_from(const glm::vec2& point) const
    {
        return Rwf * (point - S0) + Pwf;
    }

    /**
     * @brief Transforms the vector in parent frame to the local frame
              (Position of the frame origin has no effect)
     * @param vector
     * @return
     */
    glm::vec2 vector_to(const glm::vec2& vector) const
    {
        return Rfw * vector;
    }

    /**
     * @brief Transforms the vector in local frame to the parent frame
              (Position of the frame origin has no effect)
     * @param vector
     * @return
     */
    glm::vec2 vector_from(const glm::vec2& vector) const
    {
        return Rwf * vector;
    }

    /**
     * Convert from FrenetFrame to the global Cartesian frame
     * @param lon Frenet frame longitudinal trajectory
     * @param lat Frenet frame lateral trajectory
     * @param trajectory Original (reference) trajectory.  local path will be "bended" along trajectory
     * @param global_trajectory Result trajectory
     */
    static void path_to_global(const Trajectory1D& lon, const Trajectory1D& lat, const dynarray<glm::vec2>& trajectory, Trajectory2D& global_trajectory)
    {
        assert(lon.size == lat.size);

        size_t trajectory_index = 0;
        float trajectory_s = 0;

        glm::vec2 pos0(lon.x[0], lat.x[0]);
        global_trajectory.size = lon.size;
        global_trajectory.ok = lon.ok && lat.ok;

        for(size_t i = 0; i<lon.size; i++)
        {
            while (true)
            {
                if(trajectory_index >= lon.size)
                {
                    global_trajectory.size = i;
                    return;;
                }

                glm::vec2 pos(lon.x[i], lat.x[i]);
                glm::vec2 dpos(lon.dx[i], lat.dx[i]);
                glm::vec2 ddpos(lon.ddx[i], lat.ddx[i]);

                float segment_lengt = (trajectory[trajectory_index + 1] - trajectory[trajectory_index]).length();
                if (trajectory_s + segment_lengt >= lon.x[0])
                    break;

                trajectory_s += segment_lengt;
                trajectory_index++;
            }

            auto frenet = FrenetFrame(trajectory_s, trajectory[trajectory_index], trajectory[trajectory_index+1]);
            global_trajectory.x[i] = frenet.point_from(glm::vec2(lon.x[i], lat.x[i]));
            global_trajectory.dx[i] = frenet.point_from(glm::vec2(lon.dx[i], lat.dx[i]));
            global_trajectory.ddx[i] = frenet.point_from(glm::vec2(lon.ddx[i], lat.ddx[i]));
        }
    }

private:
    glm::vec2 tr;      // tangent
    glm::vec2 nr;      // normal
    glm::mat2x2 Rwf;   //local frame orientation with respect to parent frame
    glm::mat2x2 Rfw;   // parent frame orientation with respect to local frame
    glm::vec2 Pwf;     // position of the origin of the local with respect to parent frame
    glm::vec2 S0;      // covered curve length at the origin of the current frame

};

#endif //CAR_CORE_FRENET_H
