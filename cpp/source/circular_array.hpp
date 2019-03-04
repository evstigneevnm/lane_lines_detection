/*
* This file is part of the Line Detection Demo Project v 0.01c distribution. 
(https://github.com/evstigneevnm/lane_lines_detection).
* Copyright (c) 2019 Dr. Evstigneev Nikolay Mikhaylovitch.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, version 2 only.
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
* General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*/



#ifndef __GUARD_CIRCULAR_ARRAY_HPP__
#define __GUARD_CIRCULAR_ARRAY_HPP__


template<typename T>
class circular_array
{
public:
    circular_array(size_t maximum_size_):
    maximum_size(maximum_size_)
    {
        my_container.resize(maximum_size);
        current_index = 0;
        init_size = true;
    }

    circular_array()
    {
        current_index = 0;
        maximum_size = 0;
    }

    ~circular_array()
    {
        if(init_size)
        {
            my_container.clear();
        }
    }
    
    void resize(size_t maximum_size_)
    {
        maximum_size = maximum_size_;
        my_container.resize(maximum_size);
        current_index = 0;
        init_size = true;
    }

    size_t size()
    {
        return maximum_size;
    }
    void push(const T& element)
    {   
        if(init_size)
        {
            my_container[current_index] = element;
            current_index++;
            current_index = current_index%maximum_size;
        }
    }

//  at overloaded to check size
    T& operator [](int index) {
        return my_container.at(index);
    }

//  conversions
    operator const std::vector<T>&()
    {
        return my_container;
    }


private:
    size_t maximum_size = 0;
    std::vector<T> my_container;
    size_t current_index;
    bool init_size = false;

};


#endif