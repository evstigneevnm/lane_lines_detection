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

#ifndef __GUARD_SELECT_TRANSFORM_ROI_HPP__
#define __GUARD_SELECT_TRANSFORM_ROI_HPP__

/*
    Class selects and transforms ROI as well as applies fitted polynomials to the original image

*/


template <class VisLib, class Project, class Polly>
class select_transform_roi
{

    typedef          VisLib lib;
    typedef          Project project;
    typedef          Polly polly;
    typedef typename lib::real real;
    typedef typename lib::point_i point_i;
    typedef typename lib::point_r point_r;
    typedef typename lib::image image;
    typedef typename lib::vector_i vector_i;
    typedef typename lib::vector_r vector_r; 
    typedef typename lib::pixel pixel;
    typedef typename lib::rectangle rectangle;
    

public:
    select_transform_roi(lib* lib_ref_, project* proj_ref_, polly* polly_ref_left_, polly* polly_ref_right_, real factor_, int number_of_result_points_, vector_i initial_coords_):
    lib_ref(lib_ref_),
    proj_ref(proj_ref_),
    polly_ref_left(polly_ref_left_),
    polly_ref_right(polly_ref_right_),
    factor(factor_),
    number_of_result_points(number_of_result_points_)
    {
        
        if(initial_coords_.size()!=4)
            throw std::runtime_error(std::string("Incorrect number of source coordinates"));

        roi_cooridnates=initial_coords_;
        transformed_roi_cooridnates.resize(4);
    }
    select_transform_roi(lib* lib_ref_, project* proj_ref_, polly* polly_ref_left_, polly* polly_ref_right_, real factor_, int number_of_result_points_):
    lib_ref(lib_ref_),
    proj_ref(proj_ref_),
    polly_ref_left(polly_ref_left_),
    polly_ref_right(polly_ref_right_),    
    factor(factor_),
    number_of_result_points(number_of_result_points_)
    {
        transformed_roi_cooridnates.resize(4);
    }


    void set_roi_domain(vector_i initial_coords_)
    {
        roi_cooridnates = initial_coords_;
    }

    void get_roi_domain(vector_i& initial_coords_)
    {
        initial_coords_ = roi_cooridnates;
    }

    ~select_transform_roi()
    {

    }
  
    static void on_mouse(int event, int x, int y, int flags, void* param);


    void set_mouse_callback(std::string& window_name)
    {
        lib_ref->set_mouse_callback(window_name, on_mouse);
    }

    void apply_roi(vector_i& roi_, image& image_origin)
    {
        original_image_cols = image_origin.cols;
        original_image_rows = image_origin.rows;
        display_roi(image_origin);
        if(validation_needed)
        {
            construct_transformed_roi_cooridnates(original_image_cols, original_image_rows);
            proj_ref->define_homography(roi_cooridnates, transformed_roi_cooridnates);
            validation_needed = false;
            
            
        }
        roi_ = transformed_roi_cooridnates;        
        
    }

    void map_roi(const image& image_input, image& image_output)
    {
        image temp_image;
        proj_ref->apply_direct_perspective_projection(image_input, temp_image, original_image_cols, original_image_rows);

        rectangle crop_roi(transformed_roi_cooridnates[2].x,transformed_roi_cooridnates[2].y,transformed_roi_cooridnates[0].x - transformed_roi_cooridnates[2].x,transformed_roi_cooridnates[0].y-transformed_roi_cooridnates[2].y);

        //not too much to copy, and we have compiler optimization  (g++ -fno-elide-constructors must not be used!) ;-)

        image_output = temp_image(crop_roi);
    }

    void map_result_back(image& result_image)
    {
        real len_y = real(transformed_roi_cooridnates[0].y-transformed_roi_cooridnates[2].y);
        real dy = len_y/real(number_of_result_points-1);
        int shift_x = transformed_roi_cooridnates[2].x;
        int shift_y = transformed_roi_cooridnates[2].y;

        vector_r points_left;
        vector_r points_left_T;
        vector_r points_right;
        vector_r points_right_T;        

        for(int j=0;j<number_of_result_points;j++)
        {
            real point_y = dy*(j);
            real point_x_left = shift_x+polly_ref_left->get_polynomial_value(point_y);
            real point_x_right = shift_x+polly_ref_right->get_polynomial_value(point_y);

            point_r point_left_l = point_r(point_x_left, point_y);
            point_r point_right_l = point_r(point_x_right, point_y);
    
            points_left.push_back(point_left_l); 
            points_right.push_back(point_right_l);

        }
        proj_ref->apply_inverse_transform(points_left, points_left_T);
        proj_ref->apply_inverse_transform(points_right, points_right_T);

        for(int j=0;j<number_of_result_points;j++)
        {
            lib_ref->circle(result_image, (point_i)points_left_T[j], 3, pixel(255, 0, 0), 2);
            lib_ref->circle(result_image, (point_i)points_right_T[j], 3, pixel(0, 0, 255), 2);
        }        

    }


private:
    lib *lib_ref;
    project *proj_ref;
    polly *polly_ref_left;
    polly *polly_ref_right;

    real factor;
    vector_i transformed_roi_cooridnates;
    int original_image_cols;
    int original_image_rows;
    int number_of_result_points;

    static vector_i roi_cooridnates;
    static bool dragging;
    static int selected_corner_index;
    static bool validation_needed;


    void display_roi(image& image_origin)
    {
        for ( int i = 0; i < 4; ++i )
        {
            lib_ref->line(image_origin, roi_cooridnates[i], roi_cooridnates[(i + 1) % 4], pixel(0, 0, 255), 2);
            lib_ref->circle(image_origin, roi_cooridnates[i], 5, pixel(0, 255, 0), 3);
        }
    }

    void construct_transformed_roi_cooridnates(int original_image_cols, int original_image_rows)
    {
        int offset = std::max(300, int(factor*original_image_rows));
        
        transformed_roi_cooridnates[0].x = roi_cooridnates[0].x + offset;
        transformed_roi_cooridnates[0].y = original_image_rows;

        transformed_roi_cooridnates[1].x = roi_cooridnates[0].x + offset;
        transformed_roi_cooridnates[1].y = 0;

        transformed_roi_cooridnates[2].x = roi_cooridnates[3].x - offset;
        transformed_roi_cooridnates[2].y = 0;

        transformed_roi_cooridnates[3].x = roi_cooridnates[3].x - offset;
        transformed_roi_cooridnates[3].y = original_image_rows;

    }

};



template<class VisLib, class Project, class Polly>
typename select_transform_roi<VisLib, Project, Polly>::vector_i select_transform_roi<VisLib, Project, Polly>::roi_cooridnates(4);

template<class VisLib, class Project, class Polly>
bool select_transform_roi<VisLib, Project, Polly>::dragging = false;

template<class VisLib, class Project, class Polly>
int select_transform_roi<VisLib, Project, Polly>::selected_corner_index = 0;

template<class VisLib, class Project, class Polly>
bool select_transform_roi<VisLib, Project, Polly>::validation_needed = true;

template<class VisLib, class Project, class Polly>
void select_transform_roi<VisLib, Project, Polly>::on_mouse(int event, int x, int y, int flags, void* param)
    {
        if (roi_cooridnates.size() == 4)
        {
            for (int i = 0; i < 4; ++i)
            {
                if ((event == 1) & ((std::abs(roi_cooridnates[i].x - x) < 10)) & (std::abs(roi_cooridnates[i].y - y) < 10))
                {
                    selected_corner_index = i;
                    dragging = true;
                }
            }
        }

        // Action when left button is released
        if (event == 4)
        {
            dragging = false;
            validation_needed = true;
        }

        // Action when left button is pressed and mouse has moved over the window
        if ((event == 0) && dragging)
        {
            roi_cooridnates[selected_corner_index].x =  x;
            roi_cooridnates[selected_corner_index].y =  y;
            //validation_needed = true;
        }
    }


#endif