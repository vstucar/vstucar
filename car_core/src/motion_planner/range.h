//
// Created by garrus on 08.05.19.
//

#ifndef CAR_CORE_RANGE_H
#define CAR_CORE_RANGE_H

#include <iostream>
#include <vector>
using namespace std;

template <class T>
struct range;

/**
 * Returns range expression to iterate cnt values in [start; stop]
 * @tparam T
 * @param start The starting value of the sequence
 * @param stop  The end value of the sequence
 * @param cnt   Number of elements in sequence
 * @return      Magic range<T> object to use in range-based loop
 */
template<class T>
range<T> linspace(const T start, const T stop, size_t cnt)
{
    return range<T>(start, (stop - start)/(cnt - 1), cnt);
}

/**
 * Returns range expression to iterate values in [start; stop] with given step
 * NOTE: due to rounding it's not always possible to reach exactly stop value
 *       e.g. {1, 10, 0.33} give 9.91 (27 iterations) or 10.24 (28 iterations)
 * @tparam T
 * @param start The starting value of the sequence
 * @param stop  The end value of the sequence
 * @param step  Step between elements
 * @return      Magic range<T> object to use in range-based loop
 */
template<class T>
range<T> arange(const T start, const T stop, const T step)
{
    return range<T>(start, step, static_cast<size_t>((stop - start)/step + 1));
}

template <class T>
class range
{
public:

    struct value
    {
        size_t index;
        T value;
    };

    class iterator
    {
    public:
        iterator(const T start, const T step, size_t i ) :
                _start(start),
                _step(step),
                _i(i)

        {
        }

        inline __attribute__((always_inline))
        iterator& operator++()
        {
            _i++;
            return *this;
        }

        inline __attribute__((always_inline))
        bool operator!=(const iterator& other) const
        {
            return _i != other._i;
        }

        inline __attribute__((always_inline))
        value operator*() const
        {
            return value{_i, _start + _i * _step};
        }

    private:
        size_t _i;
        const T _start;
        const T _step;
    };

    const iterator begin() const
    {
        return iterator(_start, _step, 0);
    }

    const iterator end() const
    {
        return iterator(_start, _step, _cnt);
    }

    /**
     * Returns number of elements in range
     */
    size_t size() const
    {
        return _cnt;
    }


    std::vector<T> to_vector() const
    {
        std::vector<T> array(size());
        for(auto it: *this)
            array[it.index] = it.value;

        // Do copy?
        return array;
    }

private:
    range(const T start, const T step, size_t cnt) :
            _start(start),
            _step(step),
            _cnt(cnt)
    {
    }

    friend range<T> linspace<T>(const T start, const T stop, size_t cnt);
    friend range<T> arange<T>(const T start, const T stop, const T step);

    const T _start;
    const T _step;
    size_t _cnt;
};


#endif //CAR_CORE_RANGE_H
