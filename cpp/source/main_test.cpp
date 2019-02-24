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

#include <iostream>

#include <cstdlib>
#include <ctime>

#include "visual_library_wrap.hpp"
#include "projector.hpp"
#include "polynomial_lms_fit.hpp"
#include "fit_to_lanes.hpp"
#include "image_filters.hpp"
#include "video_capture.hpp"
#include "select_transform_roi.hpp"

int main(int argc, char const *argv[])
{
    //typedef types and library wrap
    typedef SCALAR_TYPE real;
    typedef vlw::visual_library_wrap<real> visual_library;
    typedef visual_library::point_i point_i;
    typedef visual_library::point_r point_r;
    typedef visual_library::image image;
    typedef visual_library::pixel pixel;
    typedef visual_library::vector_i vector_i;
    typedef visual_library::vector_r vector_r; 

    //typedef classes
    typedef projector<visual_library> projector_library;
    typedef polynomial_lms_fit<real, 3> poly_library;
    typedef image_filters<visual_library> filters_library;
    typedef fit_to_lanes<visual_library, poly_library> lanes_library;
    typedef video_capture<visual_library> video_library;
    typedef select_transform_roi<visual_library, projector_library, poly_library> roi_library;
    
    //define classes
    visual_library library;
    projector_library proj(&library);
    poly_library poly_left; //x0, q0
    poly_library poly_right; //x0, q0
    lanes_library lanes(&library, &poly_left, &poly_right, 10, 20, 150); // number of vertical windows, horizonatal window width, pixel threshold
    filters_library filters(&library);
    video_library video(&library);
    roi_library roi(&library, &proj, &poly_left, &poly_right, 0.85, 10);
    
    //main
    std::string window_origin = "Main Window";
    std::string window_roi = "ROI Window";
    std::string window_result = "Result Window";
    if(argc!=2)
    {
        printf("%s video_file_name\n", argv[0]);
        return 0;
    }
    //set windows sizes:
    int window_width = 640;
    int window_height = 360;

    std::string f_name(argv[1]);
    video.open_source(f_name);
    
    library.create_window(window_origin);
    library.create_window(window_roi);
    library.create_window(window_result);
    roi.set_mouse_callback(window_origin);


    image image_origin, image_origin_grey, image_origin_marked, image_roi, image_roi_filtered, image_result;

    //this should be configured externally!
    vector_i roi_corners;
    vector_i roi_corners_mapped(4);

    roi_corners.push_back(point_i( 67, 340 ));
    roi_corners.push_back(point_i( 273, 224 ));
    roi_corners.push_back(point_i( 363, 224 ));
    roi_corners.push_back(point_i( 621, 340 ));

    filters.set_image_size(window_width, window_height);
    roi.set_roi_domain(roi_corners);
    filters.set_region_of_interest(roi_corners);


    bool play_video = true;
    while(true)
    {
    
        if(play_video)
        {
            if(!video.read(image_origin))
            {
                video.set_begining();
                video.read(image_origin);
            }
        }

        library.resize(image_origin, window_width, window_height);
        library.copy_image(image_origin, image_origin_marked);
        library.grb_to_gray(image_origin, image_origin_grey );

        roi.apply_roi(roi_corners_mapped, image_origin_marked);
        roi.map_roi(image_origin_grey, image_roi);
        
        filters.set_region_of_interest(roi_corners_mapped);
        filters.apply_filters(image_roi, image_roi_filtered);

        lanes.construct_lanes(image_roi_filtered);

        roi.map_result_back(image_origin);

        library.show_image(window_origin, image_origin_marked);
        library.show_image(window_roi, image_roi_filtered);
        library.show_image(window_result, image_origin);



        char key = (char) library.wait_key(1);
        if((key == 'q')||(key == 27))
            break;
        else if (key == 'p')
            play_video = !play_video;
        else if (key == 'r')
            cv::imwrite( "roi.png", image_roi_filtered );


    }

    
    library.close_all_windows();
    return 0;
}