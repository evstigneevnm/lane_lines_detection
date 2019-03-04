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

#ifndef __GUARD_FIT_TO_LANES_HPP__
#define __GUARD_FIT_TO_LANES_HPP__

/*
    class takes image that was filtered and fits polynomials to the detected active pixels
    templated from a library wrapper and a polynomial class.
    The fitted data is stored in passed polynomial class references that can be used later to put results to the original image

    NOTE: as a DEMO, we are only using histogram detection and window fitting.
    One can include multiple images with different filters, their convex halls, neural networks with convolution etc, etc.

*/

#include "circular_array.hpp"

template<class VisLib, class Polly>
class fit_to_lanes: public circular_array<typename VisLib::real>
{
public:
    typedef VisLib lib;
    typedef Polly polly;        // assume that polynomials share same types.
    typedef typename lib::real real;
    typedef typename lib::point_i point_i;
    typedef typename lib::point_r point_r;
    typedef typename lib::image image;
    typedef typename lib::vector_i vector_i;
    typedef typename lib::vector_r vector_r;  
    typedef typename lib::pixel pixel;
    typedef circular_array<real> circ_array;


    fit_to_lanes(lib* lib_ref_, polly* polly_ref_left_, polly* polly_ref_right_, int number_of_windows_vertical_, int window_horizontal_, int window_points_threshold_, int avaraging_length_):
    number_of_windows_vertical(number_of_windows_vertical_),
    window_horizontal(window_horizontal_),
    window_points_threshold(window_points_threshold_),
    lib_ref(lib_ref_),
    polly_ref_left(polly_ref_left_),
    polly_ref_right(polly_ref_right_),
    avaraging_length(avaraging_length_)
    {
        hist = new histogram(*this);
        hist_points_left.resize(number_of_windows_vertical);
        hist_points_right.resize(number_of_windows_vertical);
        counter_left = 0;
        counter_right = 0;
    }
    
    ~fit_to_lanes()
    {
        delete hist;
    }
    

    
    void construct_lanes(image& original_image)
    {
        hist_size = original_image.cols;
        sum_size = original_image.rows;


        window_vertical = sum_size/number_of_windows_vertical;
        
        //int critical_size = std::max<int>(avaraging_length, sum_size*hist_size);
        int critical_size = avaraging_length;

        //TODO: possibly use better memory optimization
        if((x_points_left.size()==0)||(x_points_left.size()<critical_size))
        {
        //TODO: add some heuristics about constrains?
        //      Use lagrange multiplyers to get |derivative| \leq \varepsilon in polynomials?

            polly_ref_left->set_constrain(real(sum_size), 0.0);
            polly_ref_right->set_constrain(real(sum_size), 0.0);

            x_points_left.resize(critical_size);
            y_points_left.resize(critical_size);
            x_points_right.resize(critical_size);
            y_points_right.resize(critical_size);
        }

        hist->get_histograms(original_image);

        return_points_from_histograms();

        for(int j=0;j<number_of_windows_vertical;j++)
        {
            window(original_image, hist_points_left[j], x_points_left, y_points_left);
            window(original_image, hist_points_right[j], x_points_right, y_points_right);
        }

        
        polly_ref_left->fit(critical_size, y_points_left, x_points_left);
        polly_ref_right->fit(critical_size, y_points_right, x_points_right);

        apply_detected_lanes(original_image);

    }

private:



    class histogram
    {
    public:
        histogram(fit_to_lanes& enclosed_ref_):
        enclosed_ref(enclosed_ref_)
        {
            all_histograms.resize(enclosed_ref.number_of_windows_vertical);
            vertical_points.resize(enclosed_ref.number_of_windows_vertical);
        }
        
        ~histogram()
        {
            all_histograms.resize(0);
            vertical_points.resize(0);
        }
        
        void get_histograms(const image& original_image)
        {
            int n = enclosed_ref.number_of_windows_vertical;

            if((histogram_w.size()==0)||(histogram_w.size()!=enclosed_ref.hist_size))
                histogram_w.resize(enclosed_ref.hist_size);

            std::vector<unsigned int> indecies(n+1, enclosed_ref.sum_size/n);
            std::vector<unsigned int> sizes(n, enclosed_ref.sum_size/n);
            for(unsigned int l=0;l<enclosed_ref.sum_size/n%n;l++)
            {
                sizes[l]++;
            }
            unsigned int index_l = 0;
            for(unsigned int l=0;l<n;l++)
            {
                indecies[l] = index_l;
                index_l += sizes[l];
            }
            indecies[n]=index_l;            

            //TODO: update later with std::stream
            for(unsigned int w=0;w<n;w++)
            {
                unsigned int begin_l = indecies[w];
                unsigned int end_l = indecies[w+1];
                vertical_points[w]=(begin_l+end_l)/2;

                for(unsigned int j=0;j<enclosed_ref.hist_size;j++)
                {
                    histogram_w[j]=0;
                    real val;
                    for(unsigned int k=begin_l;k<end_l;k++)
                    {
                        histogram_w[j]+=(real)enclosed_ref.lib_ref->read_image_value_uchar(original_image, k, j);

                    }
                }

                histogram_filter(histogram_w, histogram_f);
                all_histograms[w]=histogram_f;

            }


        }

        std::vector<std::vector<real>> all_histograms;
        std::vector<int> vertical_points;
    private:
        fit_to_lanes& enclosed_ref;
        std::vector<real> histogram_w;
        std::vector<real> histogram_f;




        void histogram_filter(const std::vector<real>& hist, std::vector<real>& hist_filtered)
        {
            int size_hist = hist.size();
            int size_hist_filtered = hist_filtered.size();
            if((size_hist_filtered==0)||(size_hist_filtered!=size_hist))
                hist_filtered.resize(size_hist);

            for(int j=2;j<size_hist-2;j++)
            {
                hist_filtered[j]=(0.175*hist[j+2]+0.2*hist[j+1]+0.25*hist[j]+0.2*hist[j-1]+0.175*hist[j-2]);
            }
            real hist_filtered_max = std::max<real>(*std::max_element(hist_filtered.begin(), hist_filtered.end()),real(1));

            for_each( hist_filtered.begin(),hist_filtered.end(), 
                [hist_filtered_max](real& x){ x/=hist_filtered_max; }  );

        }

    };



    void return_points_from_histograms()
    {
        //TODO: a better way to do it is to feat LMS splies and get local extrema
        //      Then use argmax points to tune windows.

        int minimal_distance_between_lanes = hist_size/3;
        int j = 0;
        for(auto& x: hist->all_histograms)
        {
            int first = -1;
            int second = -1;
            auto result1 = std::max_element(x.begin(), x.end());
            first = std::distance(x.begin(), result1);
            if(*result1==0)
                first=-1;

            int lim_low = std::max(0,first-minimal_distance_between_lanes);
            int lim_hi = std::min(hist_size,first+minimal_distance_between_lanes);
            for(int k = lim_low; k < lim_hi;k++)
            {
                x[k]=real(0);
            }

            auto result2 = std::max_element(x.begin(), x.end());
            second = std::distance(x.begin(), result2);
            if(*result2==0)
                second=-1;

            if(first>second)
                std::swap<int>(first, second);

            //heuristics! I hate'em.
            if(second==-1)
                if(first>hist_size/2)
                    std::swap<int>(first, second);
            if(first==-1)
                if(second<hist_size/2)
                    std::swap<int>(first, second);
            


            hist_points_left[j] = point_r(hist->vertical_points[j], first);
            hist_points_right[j] = point_r(hist->vertical_points[j], second);
            j++;

        }

    }


    void window(const image& original_image, point_i& hist_points, circ_array& x_points, circ_array& y_points)
    {
        int center_horizontal = hist_points.y;
        int center_vertical = hist_points.x;
        if(center_horizontal>-1)
        {
            unsigned int wind_h_start = std::max<int>(center_horizontal - window_horizontal/2, 0);
            unsigned int wind_v_start = std::max<int>(center_vertical -  window_vertical/2, 0);
            unsigned int wind_h_end = std::min<int>(center_horizontal + window_horizontal/2, hist_size);
            unsigned int wind_v_end = std::min<int>(center_vertical +  window_vertical/2, sum_size);

            //printf("(%i,%i)->[%i,%i]X[%i,%i]\n", center_horizontal, center_vertical, wind_h_start, wind_h_end, wind_v_start, wind_v_end);
            for(int j=wind_h_start;j<wind_h_end;j++)
            {
                for(int k=wind_v_start;k<wind_v_end;k++)
                {
                    unsigned char val = lib_ref->read_image_value_uchar(original_image, k, j);
                    if(val>(unsigned char)window_points_threshold)           
                    {
                        x_points.push(real(j));
                        y_points.push(real(k));
                    }

                }
            }
        }
        
    }

    void apply_detected_lanes(image& resulting_image)
    {

        
        for(int j=0;j<number_of_windows_vertical;j++)
        {
            int point_y = int(hist->vertical_points[j]);
            int point_x_left = int( polly_ref_left->get_polynomial_value(real(point_y)) );
            int point_x_right = int( polly_ref_right->get_polynomial_value(real(point_y)) );
            point_i point_left_l = point_i(point_x_left,point_y);
            point_i point_right_l = point_i(point_x_right,point_y);

            lib_ref->circle(resulting_image, point_left_l, 5, pixel(255, 0, 0), 3);
            lib_ref->circle(resulting_image, point_right_l, 5, pixel(255,0 ,0), 3);

        }       
    }

    /*
    
    some good music used:
    
    - Arcade Messiah
    - Distant Dream
    - Toundra
    - Blackwaves
    - Blood Tsunami
    - Tool

    */

    int number_of_windows_vertical;
    int window_vertical;
    int window_horizontal;
    int window_points_threshold;
    int hist_size;
    int sum_size;
    int counter_left = 0;
    int counter_right = 0;
    int avaraging_length;

    lib *lib_ref;
    polly *polly_ref_left;
    polly *polly_ref_right;
    histogram *hist;
    std::vector<point_i> hist_points_left;
    std::vector<point_i> hist_points_right;

    circ_array x_points_left;
    circ_array y_points_left;
    circ_array x_points_right;
    circ_array y_points_right;


    image return_image;



};


#endif