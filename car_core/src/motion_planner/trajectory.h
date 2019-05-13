//
// Created by garrus on 14.05.19.
//

#ifndef CAR_CORE_TRAJECTORY_H
#define CAR_CORE_TRAJECTORY_H


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

#endif //CAR_CORE_TRAJECTORY_H
