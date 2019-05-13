//
// Created by garrus on 07.05.19.
//

#ifndef CAR_CORE_DYNARRAY_H
#define CAR_CORE_DYNARRAY_H

/**
 * @brief Simple C++-style wrapper for plain dynamic array
 * @tparam T
 */
template <class T>
class dynarray
{
public:

    explicit dynarray(size_t size)
    {
        _array = new T[size];
        _size = size;
    }

    dynarray(std::initializer_list<T> list)
            : _array(list)
    {
        _size = list.size();
    }

    dynarray(dynarray&& other)
    {
        _array = other._array;
        _size = other._size;
        other._array = nullptr;
        other._size = 0;
    }

    dynarray(const dynarray& other) = delete;


    T& operator[](size_t index){return _array[index]; }
    const T& operator[](size_t index) const {return _array[index]; }
    const size_t size() const { return _size; }

private:
    T* _array;
    size_t _size;
};


#endif //CAR_CORE_DYNARRAY_H
