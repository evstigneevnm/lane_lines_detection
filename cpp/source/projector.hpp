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

#ifndef __GUARD_PROJECTOR_HPP__
#define __GUARD_PROJECTOR_HPP__

template<class VisLib>
class projector
{
public:
    typedef VisLib lib;
    typedef typename lib::real real;
    typedef typename lib::point_i point_i;
    typedef typename lib::point_r point_r;
    typedef typename lib::image image;
    typedef typename lib::vector_i vector_i;
    typedef typename lib::vector_r vector_r;    

    
    projector(lib* local_lib_): 
    is_allocated(false), 
    local_lib(local_lib_)
    {

    }

    ~projector()
    {
        if(is_allocated)
        {
            M.release();
            Minv.release();
        }

    }

    void define_homography(const point_i& source_points, const point_i& destination_points)
    {
        M = local_lib->get_homography(source_points, destination_points);
        Minv = local_lib->get_homography(destination_points, source_points);
        is_allocated = true;
    }

    void define_homography(const point_r& source_points, const point_r& destination_points)
    {
        M = local_lib->get_homography(source_points, destination_points);
        Minv = local_lib->get_homography(destination_points, source_points);

        is_allocated = true;
    }

    void define_perspective_transform(const point_i& source_points, const point_i& destination_points)
    {
        M = local_lib->get_perspective_transform(source_points, destination_points);
        Minv = local_lib->get_perspective_transform(destination_points, source_points);

        is_allocated = true;
    }

    void define_perspective_transform(const point_r& source_points, const point_r& destination_points)
    {
        M = local_lib->get_perspective_transform(source_points, destination_points);
        Minv = local_lib->get_perspective_transform(destination_points, source_points);
        is_allocated = true;
    }
    
//
    void define_homography(const vector_i& source_points, const vector_i& destination_points)
    {
        M = local_lib->get_homography(source_points, destination_points);
        Minv = local_lib->get_homography(destination_points, source_points);

        is_allocated = true;
    }

    void define_homography(const vector_r& source_points, const vector_r& destination_points)
    {
        M = local_lib->get_homography(source_points, destination_points);
        Minv = local_lib->get_homography(destination_points, source_points);

        is_allocated = true;
    }

    void define_perspective_transform(const vector_i& source_points, const vector_i& destination_points)
    {
        M = local_lib->get_perspective_transform(source_points, destination_points);
        Minv = local_lib->get_perspective_transform(destination_points, source_points);

        is_allocated = true;
    }

    void define_perspective_transform(const vector_r& source_points, const vector_r& destination_points)
    {
        M = local_lib->get_perspective_transform(source_points, destination_points);
        Minv = local_lib->get_perspective_transform(destination_points, source_points);

        is_allocated = true;
    }


    void apply_direct_perspective_projection(const image& original_image, image& warped_image, int width, int height)
    {
        local_lib->warp_perspective(original_image, warped_image, M, width, height);
    }
    void apply_inverse_perspective_projection(const image& original_image, image& warped_image, int width, int height)
    {
        local_lib->warp_perspective(original_image, warped_image, Minv, width, height);
    }

    void apply_direct_transform(const vector_r& src, vector_r& dst)
    {
        local_lib->perspective_transform(src, dst, M);
    }
 
 
    void apply_inverse_transform(const vector_r& src, vector_r& dst)
    {
        local_lib->perspective_transform(src, dst, Minv);
    } 

 

private:
    // link to library wrap

    lib *local_lib;
    // TODO:those are simple 3X3 real matrices. 
    //      Changes these to real matrices, not library ''images''?
    //      maybe use thing copy in wrap? No time for now...
    image M;
    image Minv;
    // stupid flag
    bool is_allocated;
    
};

#endif