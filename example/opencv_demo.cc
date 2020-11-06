/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.
This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the Regents of The University of Michigan.
*/

#include <iostream>

#include "opencv2/opencv.hpp"
#include "opencv2/video.hpp"


extern "C" {
#include "apriltag.h"
#include "tag36h11.h"
#include "tag25h9.h"
#include "tag16h5.h"
#include "tagCircle21h7.h"
#include "tagCircle49h12.h"
#include "tagCustom48h12.h"
#include "tagStandard41h12.h"
#include "tagStandard52h13.h"
#include "common/getopt.h"
}

using namespace std;
using namespace cv;


int main(int argc, char *argv[])
{
    getopt_t *getopt = getopt_create();

    getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
    getopt_add_bool(getopt, 'd', "debug", 0, "Enable debugging output (slow)");
    getopt_add_bool(getopt, 'q', "quiet", 0, "Reduce output");
    getopt_add_string(getopt, 'f', "family", "tag36h11", "Tag family to use");
    getopt_add_int(getopt, 't', "threads", "1", "Use this many CPU threads");
    getopt_add_double(getopt, 'x', "decimate", "2.0", "Decimate input image by this factor");
    getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input");
    getopt_add_bool(getopt, '0', "refine-edges", 1, "Spend more time trying to align edges of tags");
    getopt_add_double(getopt, 'i', "input-scale", "1.0", "Resize input based on this scale. Values smaller than 1.0 increase performance.");
    getopt_add_int(getopt, 'l', "line-thickness", "1", "Draw line thickness");
    getopt_add_double(getopt, 'n', "font-scale", "2.0", "Text font scale");
    getopt_add_string(getopt, 'o', "output-file", "output.mp4", "Video output file of the detection.");

    if (!getopt_parse(getopt, argc, argv, 1) ||
            getopt_get_bool(getopt, "help")) {
        printf("Usage: %s [options] [<video_file>]\n", argv[0]);
        getopt_do_usage(getopt);
        exit(0);
    }

    const zarray_t *inputs = getopt_get_extra_args(getopt);

    // Initialize camera
    VideoCapture cap;
    if (zarray_size(inputs) == 0) {
        cap = VideoCapture(0);
    } else if (zarray_size(inputs) == 1) {
        char *path;
        zarray_get(inputs, 0, &path);
        printf("loading %s\n", path);
        cap = VideoCapture(path);
    } else {
        cerr << "Only supports one video file at a time" << endl;
        return -1;
    }
    if (!cap.isOpened()) {
        cerr << "Couldn't open video capture device or file" << endl;
        return -1;
    }

    // Initialize tag detector with options
    apriltag_family_t *tf = NULL;
    const char *famname = getopt_get_string(getopt, "family");
    if (!strcmp(famname, "tag36h11")) {
        tf = tag36h11_create();
    } else if (!strcmp(famname, "tag25h9")) {
        tf = tag25h9_create();
    } else if (!strcmp(famname, "tag16h5")) {
        tf = tag16h5_create();
    } else if (!strcmp(famname, "tagCircle21h7")) {
        tf = tagCircle21h7_create();
    } else if (!strcmp(famname, "tagCircle49h12")) {
        tf = tagCircle49h12_create();
    } else if (!strcmp(famname, "tagStandard41h12")) {
        tf = tagStandard41h12_create();
    } else if (!strcmp(famname, "tagStandard52h13")) {
        tf = tagStandard52h13_create();
    } else if (!strcmp(famname, "tagCustom48h12")) {
        tf = tagCustom48h12_create();
    } else {
        printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
        exit(-1);
    }


    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = getopt_get_double(getopt, "decimate");
    td->quad_sigma = getopt_get_double(getopt, "blur");
    td->nthreads = getopt_get_int(getopt, "threads");
    td->debug = getopt_get_bool(getopt, "debug");
    td->refine_edges = getopt_get_bool(getopt, "refine-edges");
    double input_scale = getopt_get_double(getopt, "input-scale");
    double font_scale = getopt_get_double(getopt, "font-scale");
    int line_thickness = getopt_get_double(getopt, "line-thickness");
    const char *output_file = getopt_get_string(getopt, "output-file");

    double src_fps = 25.0;
    try {
        src_fps = cap.get(cv::CAP_PROP_FPS);
    }
    catch (...) {
        cerr << " Can't get FPS of source videofile. For output video FPS = 25 by default. \n";
    }
    
    Mat frame, gray, frame_resized;
    Size scaled_size = Size((int) cap.get(CAP_PROP_FRAME_WIDTH) * input_scale,    // Acquire input size
                  (int) cap.get(CAP_PROP_FRAME_HEIGHT) * input_scale);
    VideoWriter out = VideoWriter(output_file, VideoWriter::fourcc('H','2','6','4'), src_fps, scaled_size);
    while (true) {
        if (!cap.read(frame))
            break;
        resize(frame, frame, scaled_size);
        cvtColor(frame, gray, COLOR_BGR2GRAY);

        // Make an image_u8_t header for the Mat data
        image_u8_t im = { .width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data
        };

        zarray_t *detections = apriltag_detector_detect(td, &im);

        // Draw detection outlines
        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            line(frame, Point(det->p[0][0], det->p[0][1]),
                     Point(det->p[1][0], det->p[1][1]),
                     Scalar(0, 0xff, 0), line_thickness);
            line(frame, Point(det->p[0][0], det->p[0][1]),
                     Point(det->p[3][0], det->p[3][1]),
                     Scalar(0, 0, 0xff), line_thickness);
            line(frame, Point(det->p[1][0], det->p[1][1]),
                     Point(det->p[2][0], det->p[2][1]),
                     Scalar(0xff, 0, 0), line_thickness);
            line(frame, Point(det->p[2][0], det->p[2][1]),
                     Point(det->p[3][0], det->p[3][1]),
                     Scalar(0xff, 0, 0), line_thickness);

            stringstream ss;
            ss << det->id;
            String text = ss.str();
            int fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
            int baseline;
            Size textsize = getTextSize(text, fontface, font_scale, 1,
                                            &baseline);
            putText(frame, text, Point(det->c[0]-textsize.width/2,
                                       det->c[1]+textsize.height/2),
                    fontface, font_scale, Scalar(0xff, 0x99, 0), 2);
        }
        apriltag_detections_destroy(detections);

        namedWindow("Tag Detections", WINDOW_KEEPRATIO);
        out.write(frame);
        imshow("Tag Detections", frame);
        if (waitKey(1) >= 0)
            break;
    }

    out.release();
    cap.release();

    apriltag_detector_destroy(td);

    if (!strcmp(famname, "tag36h11")) {
        tag36h11_destroy(tf);
    } else if (!strcmp(famname, "tag25h9")) {
        tag25h9_destroy(tf);
    } else if (!strcmp(famname, "tag16h5")) {
        tag16h5_destroy(tf);
    } else if (!strcmp(famname, "tagCircle21h7")) {
        tagCircle21h7_destroy(tf);
    } else if (!strcmp(famname, "tagCircle49h12")) {
        tagCircle49h12_destroy(tf);
    } else if (!strcmp(famname, "tagStandard41h12")) {
        tagStandard41h12_destroy(tf);
    } else if (!strcmp(famname, "tagStandard52h13")) {
        tagStandard52h13_destroy(tf);
    } else if (!strcmp(famname, "tagCustom48h12")) {
        tagCustom48h12_destroy(tf);
    }


    getopt_destroy(getopt);

    return 0;
}
