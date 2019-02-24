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

#ifndef __GUARD__VISUAL_LIBRARY_WRAP_HPP__
#define __GUARD__VISUAL_LIBRARY_WRAP_HPP__

/*


Visual Library Wrap! 
For now it wraps OpenCV functions for CPU.

In general we don't care what library is used. 
we just make a wrap so that in the future it can be easily changed
we also export data types from here, so we can use CUDA, OpenCL, FPGA etc if needed (not implemented now).


*/

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

namespace vlw{

template <typename T>
class visual_library_wrap
{
public:
    typedef T real;
    typedef cv::Mat image;
    typedef cv::Point_<int> point_i;
    typedef cv::Point_<T> point_r;
    typedef std::vector<point_i> vector_i;
    typedef std::vector<point_r> vector_r;
    typedef cv::Scalar_<T> pixel; //Scalar (blue_ component, green_ component, red_ component, alpha_ component)
    typedef cv::VideoCapture video_stream;
    typedef cv::Rect rectangle;




    visual_library_wrap()
    {

    }

    ~visual_library_wrap()
    {

    }
 
    T read_image_value(const image& im, int j, int k)
    {
        return im.at<T>(j,k);
    }
    unsigned char read_image_value_uchar(const image& im, int j, int k)
    {
        return im.at<unsigned char>(j,k);
    }
    unsigned int read_image_value_uint(const image& im, int j, int k)
    {
        return im.at<unsigned int>(j,k);
    }
    char read_image_value_char(const image& im, int j, int k)
    {
        return im.at<char>(j,k);
    }
    int read_image_value_int(const image& im, int j, int k)
    {
        return im.at<int>(j,k);
    }    
    uint64 read_image_value_uint64(const image& im, int j, int k)
    {
        return im.at<uint64>(j,k);
    }
            
    image read_image(std::string f_name)
    {
        return cv::imread( f_name );    
    }

    void create_window_opengl(const std::string window_name)
    {
        cv::namedWindow(window_name, cv::WINDOW_OPENGL);
    }

    void create_window(const std::string& window_name)
    {
        cv::namedWindow(window_name, cv::WINDOW_NORMAL);
    }

    void destroy_window(const std::string window_name)
    {
        cv::destroyWindow(window_name);
    }

    void show_image(const std::string window_name, image& image_id)
    {
        cv::imshow(window_name, image_id);
    }

    int wait_key_press(int delay=0)
    {
        return cv::waitKey(delay);
    }
    image get_homography(const point_r& input_array, const point_r& output_array)
    {
        return cv::findHomography((cv::InputArray)input_array,(cv::InputArray)output_array);
    }
    image get_homography(const point_i& input_array, const point_i& output_array)
    {
        return cv::findHomography((cv::InputArray)input_array, (cv::InputArray)output_array);
    }
    image get_perspective_transform(const point_r& input_array, const point_r& output_array)
    {
        return cv::getPerspectiveTransform((cv::InputArray)input_array, (cv::InputArray)output_array);    
    }

    image get_perspective_transform(const point_i& input_array, const point_i& output_array)
    {
        return cv::getPerspectiveTransform((cv::InputArray)input_array, (cv::InputArray)output_array);    
    }

    image get_homography(const vector_r& input_array, const vector_r& output_array)
    {
        return cv::findHomography((cv::InputArray)input_array,(cv::InputArray)output_array);
    }
    image get_homography(const vector_i& input_array, const vector_i& output_array)
    {
        return cv::findHomography((cv::InputArray)input_array, (cv::InputArray)output_array);
    }
    image get_perspective_transform(const vector_r& input_array, const vector_r& output_array)
    {
        return cv::getPerspectiveTransform((cv::InputArray)input_array, (cv::InputArray)output_array);    
    }

    image get_perspective_transform(const vector_i& input_array, const vector_i& output_array)
    {
        return cv::getPerspectiveTransform((cv::InputArray)input_array, (cv::InputArray)output_array);    
    }


    void warp_perspective(const image& original_image, image& warped_image, image &H, int width, int height)
    {
        cv::Size warped_image_size = cv::Size(width, height);
        warpPerspective(original_image, warped_image, H, warped_image_size);
    }

    void fill_convex_poly(image& img, const point_i* pts, int npts, const pixel& color, int lineType=8, int shift=0)
    {
        cv::fillConvexPoly(img, pts, npts, color, lineType, shift);
    }

    void fill_convex_poly(image& img, const vector_i pts, const pixel& color)
    {
        cv::fillConvexPoly(img, pts, color);
    }  

    void bitwise_and(const image& source_image_1, const image& source_image_2, image& output_image, image& mask)
    {
        cv::bitwise_and((cv::InputArray) source_image_1, (cv::InputArray) source_image_2, (cv::OutputArray) output_image, (cv::InputArray) mask);
    }

    void copy_image(const image& source, image& destination)
    {
        source.copyTo(destination);
    }

    void threshold(const image& src, image& dst, T thresh, T maxval, int type = cv::THRESH_BINARY)
    {
        cv::threshold((cv::InputArray) src, (cv::OutputArray) dst, thresh, maxval, type);
    }

    void gaussian_blur(const image& src, image& dst, int kernel_width, int kernel_height, double sigmaX, double sigmaY=0, int borderType=cv::BORDER_DEFAULT  )
    {   
        cv::Size kernel_size = cv::Size(kernel_width, kernel_height);
        cv::GaussianBlur((cv::InputArray) src, (cv::OutputArray) dst, kernel_size, sigmaX, sigmaY, borderType);
    }

    void sobel(const image& src, image& dst, int dx, int dy, int kernel_size=3, int ddepth=-1, double scale=1, double delta=0, int borderType = cv::BORDER_REPLICATE  )
    {
        cv::Sobel((cv::InputArray) src, (cv::OutputArray) dst, ddepth, dx, dy, kernel_size=3, scale=1, delta=0, borderType);
    }

    void close_all_windows()
    {
        cv::destroyAllWindows();
    }

    void set_mouse_callback(const std::string& window_name, cv::MouseCallback(on_mouse))
    {
        cv::setMouseCallback(window_name, on_mouse, 0);

    }

    int wait_key(int delay)
    {
        return cv::waitKey(delay);
    }

    void resize(image& frame, int width, int height)
    {
        cv::Size warped_image_size = cv::Size(width, height);
        cv::resize(frame, frame, warped_image_size, 0, 0, cv::INTER_LINEAR);
    }
    void resize(const image& frame_int, image& frame_out, int width, int height)
    {
        cv::Size warped_image_size = cv::Size(width, height);
        cv::resize(frame_int, frame_out, warped_image_size, 0, 0, cv::INTER_LINEAR);
    }
    void grb_to_gray(const image& input, image& output)
    {
        cv::cvtColor(input, output, cv::COLOR_RGB2GRAY);
    }

    void calculate_histogram(const image& image_source, int hist_size, image& hist)
    {
        cv::calcHist(&image_source, 1, 0, cv::Mat(), hist, 1, &hist_size, 0);
    }

    void perspective_transform(const vector_r& src, vector_r& dst, image& M)
    {
        cv::perspectiveTransform( (cv::InputArray) src, (cv::OutputArray) dst,  (cv::InputArray) M); 
    }

    void perspective_transform(const point_r& src, point_r& dst, image& M)
    {
        cv::perspectiveTransform( (cv::InputArray) src, (cv::OutputArray) dst,  (cv::InputArray) M); 
    }

    void circle(const image& resulting_image, const point_i& points, int width, const pixel& color, int thickness)
    {
        cv::circle(resulting_image, points, width, color, thickness);
    }

    void line(const image& image_origin, const point_i& point_1, const point_i& point_2, const pixel& color, int thickness)
    {
        cv::line(image_origin, point_1, point_2, color, thickness);
    }

};


}

#endif