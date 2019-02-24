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

#ifndef __GUARD_IMAGE_FILTERS_HPP__
#define __GUARD_IMAGE_FILTERS_HPP__

/*
    this struct contains all filters and manipulations that are needed to convert original image to the one that can be used to fit polinomials into lanes
    templated from visual library wrapper it is not important what kind of library is used.
    
    NOTE: THIS IS JUST A DEMO. IN REALITY ONE NEEDS WAY MORE FILITERS AND IMAGE CHENNELS TO PERFORM HIGH QUALLITY DETECTION!!!
    one can use:
    1. different colour chennels
    2. various heuristics related to the object orientation
    3. possible feed-back control of filter parameters
    4. simple neuro-nets, since mapped image is relatively simple
    etc.

*/

template <class VisLib>
class image_filters
{
public:
    typedef          VisLib lib;
    typedef typename lib::real real;
    typedef typename lib::point_i point_i;
    typedef typename lib::point_r point_r;
    typedef typename lib::image image;
    typedef typename lib::vector_i vector_i;
    typedef typename lib::vector_r vector_r; 
    typedef typename lib::pixel pixel;

    image_filters(lib* lib_ref_, int original_image_rows_, int original_image_cols_):
    lib_ref(lib_ref_),
    original_image_rows(original_image_rows_),
    original_image_cols(original_image_cols_),
    made_mask(false)
    {
        mask = new image(original_image_rows, original_image_cols, CV_8U);
        made_mask=true;
    }
    
    image_filters(lib* lib_ref_):
    lib_ref(lib_ref_),
    made_mask(false)
    {
       

    }

    ~image_filters()
    {
        if(made_mask)
            delete mask;
    }

    void set_image_size(int original_image_rows_, int original_image_cols_)
    {
        original_image_rows = original_image_rows_;
        original_image_cols = original_image_cols_;
        mask = new image(original_image_cols, original_image_rows, CV_8U);
        made_mask=true;
    }

    void set_region_of_interest(const vector_i& points_mask_)
    {
        
        *mask = pixel(0, 0, 0);
        lib_ref->fill_convex_poly(*mask, points_mask_, pixel(255, 255, 255));

    }

    void set_threshold(int threshold_0_, int threshold_1_)
    {
        threshold_0=threshold_0_;
        threshold_1=threshold_1_;
    }

    
    void apply_filters(const image& input_image, image& output_image)
    {

        image image_wrap_1;
        lib_ref->gaussian_blur(input_image, output_image, 3, 3, 1.0);
        lib_ref->sobel(output_image, image_wrap_1, 1, 0, 3);
        lib_ref->threshold(image_wrap_1, output_image,  threshold_1, 255);
    }


private:
    lib* lib_ref;
    image *mask;
    int original_image_rows;
    int original_image_cols;
    int threshold_0 = 110; //using std=c++11
    int threshold_1 = 110;
    bool made_mask;
    
};



#endif