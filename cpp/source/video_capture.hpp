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

#ifndef __GUARD_VIDEO_CAPTURE_HPP__
#define __GUARD_VIDEO_CAPTURE_HPP__

/*

working with video
currently only video reading is implemented

*/


template <class VisLib>
class video_capture
{
public:
    typedef          VisLib lib;
    typedef typename lib::image image;
    typedef typename lib::video_stream video_stream;


    video_capture(lib* lib_ref_):
    lib_ref(lib_ref_)
    {
        cap = new video_stream;
    }

    video_capture(lib* lib_ref_, std::string device_name):
    lib_ref(lib_ref_)
    {
        cap = new video_stream;
        open_stream(device_name);
    }

    video_capture(lib* lib_ref_, int device_id):
    lib_ref(lib_ref_)
    {
        cap = new video_stream;
        open_stream(device_id);     
    }


    ~video_capture()
    {
        close_source();
        delete cap;
    }


    void open_source(int device_id)
    {
        open_stream(device_id);   
    }

    void open_source(std::string device_name)
    {
        open_stream(device_name);   
    }

    void close_source()
    {
        if(video_stream_opened)
            cap->release();
    }

    bool read(image& frame)
    {
        cap->read(frame);
        
        if(frame.empty())
            return false;
        else
            return true;
    }
    void set_begining()
    {
        cap->set(1, 0);
    }


private: 
    lib *lib_ref;
    video_stream *cap;
    bool video_stream_opened = false;


    void open_stream(int device_id)
    {
        cap->open(device_id);
        video_stream_opened = true;
        if(!cap->isOpened())
           throw std::runtime_error(std::string("Error opening video stream or file"));             
    }

    void open_stream(std::string device_name)
    {
        cap = new video_stream(device_name);
        video_stream_opened = true;
        if(!cap->isOpened())
           throw std::runtime_error(std::string("Error opening video stream or file"));            
    }


};


#endif