// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include <k4a/k4a.h>
#include <k4arecord/playback.h>
#include <string>
#include "transformation_helpers.h"
#include "turbojpeg.h"

#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
//#include <opencv2/core.hpp>
//#include <opencv2/imgcodecs.hpp> 
//#include <opencv2/highgui/highgui.hpp>

//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/core/core.hpp>
//#include <opencv/cv.hpp>

//#include <opencv2/imgproc.hpp>
//#include <opencv2/core.hpp>
//#include <opencv2/highgui.hpp>
//#include <opencv2/opencv.hpp>

static bool point_cloud_depth_to_color(k4a_transformation_t transformation_handle,
                                       const k4a_image_t depth_image,
                                       const k4a_image_t color_image,
                                       const char * depth_filename,
                                       const char * color_filename)
{
    // transform color image into depth camera geometry
    int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
    int color_image_height_pixels = k4a_image_get_height_pixels(color_image);
    k4a_image_t transformed_depth_image = NULL;
    FILE * pFile;
    uint8_t * buffer;
    uint32_t size;

    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                                                 color_image_width_pixels,
                                                 color_image_height_pixels,
                                                 color_image_width_pixels * (int)sizeof(uint16_t),
                                                 &transformed_depth_image))
    {
        printf("Failed to create transformed depth image\n");
        return false;
    }

    if (K4A_RESULT_SUCCEEDED !=
        k4a_transformation_depth_image_to_color_camera(transformation_handle, depth_image, transformed_depth_image))
    {
        printf("Failed to compute transformed depth image\n");
        return false;
    }

    buffer = k4a_image_get_buffer(color_image);
    size = k4a_image_get_size(color_image);
    pFile = fopen (color_filename, "w");

    //if (pFile!=NULL)
    //{
        fwrite(buffer, 1, size, pFile);
        fclose (pFile);
    //}

    buffer = k4a_image_get_buffer(transformed_depth_image);
    //size = k4a_image_get_size(transformed_depth_image);
    //pFile = fopen (depth_filename, "w");

    //if (pFile!=NULL)
    //{
        //fwrite(buffer, 1, size, pFile);
        //fclose (pFile);
    //}

    cv::imwrite(depth_filename,  cv::Mat(color_image_height_pixels, color_image_width_pixels, CV_16UC1, buffer));

    k4a_image_release(transformed_depth_image);

    return true;
}

// Timestamp in milliseconds. Defaults to 1 sec as the first couple frames don't contain color
static int playback(char *input_path, const char * output_path)
{
    int returnCode = 1;
    k4a_playback_t playback = NULL;
    k4a_calibration_t calibration;
    k4a_transformation_t transformation = NULL;
    k4a_capture_t capture = NULL;
    k4a_image_t depth_image = NULL;
    k4a_image_t color_image = NULL;

    k4a_result_t result;
    k4a_stream_result_t stream_result;

    // Open recording
    result = k4a_playback_open(input_path, &playback);
    if (result != K4A_RESULT_SUCCEEDED || playback == NULL)
    {
        printf("Failed to open recording %s\n", input_path);
        return returnCode;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_playback_get_calibration(playback, &calibration))
    {
        printf("Failed to get calibration\n");
        return returnCode;
    }

    transformation = k4a_transformation_create(&calibration);

    for (uint32_t i = 0; i < 100; i++) {
        stream_result = k4a_playback_get_next_capture(playback, &capture);
        //printf("%d, %d, %x\n", i, stream_result, capture);
        if (stream_result != K4A_STREAM_RESULT_SUCCEEDED || capture == NULL) //put here checking
        {
            if (stream_result == K4A_STREAM_RESULT_EOF) {
                returnCode = 0;
            }
            else {
                printf("Failed to fetch frame\n");
            }
            break;
        }

        // Fetch color and depth frames
        color_image = k4a_capture_get_color_image(capture);
        if (color_image == 0) {
            k4a_capture_release(capture);
            continue;
        }

        depth_image = k4a_capture_get_depth_image(capture);
        if (depth_image == 0) {
            k4a_image_release(color_image);
            k4a_capture_release(capture);
            continue;
        }
        
        uint64_t color_timestamp = k4a_image_get_device_timestamp_usec(color_image);
        char color_filename [1024];
        sprintf (color_filename, "%s/color/%012ld.jpg", output_path, color_timestamp);
        
        uint64_t depth_timestamp = k4a_image_get_device_timestamp_usec(depth_image);
        char depth_filename [1024];
        sprintf (depth_filename, "%s/depth/%012ld.png", output_path, depth_timestamp);

        // Compute color point cloud by warping depth image into color camera geometry
        if (point_cloud_depth_to_color(transformation, depth_image, color_image, depth_filename, color_filename) == false)
        {
            printf("Failed to transform depth to color\n");
            break;
        }

        if (depth_image != NULL)
        {
            k4a_image_release(depth_image);
        }
        if (color_image != NULL)
        {
            k4a_image_release(color_image);
        }
        if (capture != NULL)
        {
            k4a_capture_release(capture);
        }
    }

    if (playback != NULL)
    {
        k4a_playback_close(playback);
    }
    if (transformation != NULL)
    {
        k4a_transformation_destroy(transformation);
    }

    return returnCode;
}

int main(int argc, char **argv)
{
    int returnCode = 0;

    if (argc != 3)
    {
        printf("Usage: mrob_images_extractor <filename.mkv> output_path\n");
    }
    else
    {
        returnCode = playback(argv[1], argv[2]);
    }

    return returnCode;
}
