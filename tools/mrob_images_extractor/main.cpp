// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include <k4a/k4a.h>
#include <k4arecord/playback.h>
#include <string>
#include "transformation_helpers.h"
#include "turbojpeg.h"

#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

#include <chrono>

static int playback(char *input_path, const char * output_path)
{
    int return_code = 1;
    k4a_playback_t playback = NULL;

    k4a_calibration_t calibration;
    k4a_transformation_t transformation = NULL;
    k4a_calibration_camera_t calib_color;
    struct k4a_calibration_intrinsic_parameters_t::_param param;

    k4a_capture_t capture = NULL;
    k4a_image_t depth_image = NULL;
    k4a_image_t color_image = NULL;
    k4a_image_t transformed_depth_image = NULL;

    k4a_result_t result;
    k4a_stream_result_t stream_result;

    uint64_t color_timestamp;
    uint64_t depth_timestamp;
    char color_filename [1024];
    char depth_filename [1024];
    uint32_t i;

    int color_image_width_pixels;
    int color_image_height_pixels;
    uint8_t * buffer;
    uint32_t size;

    cv::Mat distortion;
    cv::Matx33d matrix;
    cv::Mat img_array;
    cv::Mat img_array_undistorted;
    std::vector<int> compression_params;

    compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
    compression_params.push_back(96);

    // Open recording
    result = k4a_playback_open(input_path, &playback);
    if (result != K4A_RESULT_SUCCEEDED || playback == NULL)
    {
        printf("Failed to open recording %s\n", input_path);
        return return_code;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_playback_get_calibration(playback, &calibration))
    {
        printf("Failed to get calibration\n");
        return return_code;
    }

    transformation = k4a_transformation_create(&calibration);
    
    calib_color = calibration.color_camera_calibration;
    param = calib_color.intrinsics.parameters.param;
    distortion = (cv::Mat_<double>(8,1) << param.k1, param.k2, param.p1, param.p2, param.k3, param.k4, param.k5, param.k6);
    matrix = cv::Matx33d(param.fx, 0.0, param.cx, 0.0, param.fy, param.cy, 0.0, 0.0, 1.0);

    color_image_width_pixels = calib_color.resolution_width;
    color_image_height_pixels = calib_color.resolution_height;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                                                 color_image_width_pixels,
                                                 color_image_height_pixels,
                                                 color_image_width_pixels * (int)sizeof(uint16_t),
                                                 &transformed_depth_image))
    {
        printf("Failed to create transformed depth image\n");
        return return_code;
    }
    
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    for (i = 0; i < 2592000; i++) { // 24h*3600s*30fps = 2592000
        stream_result = k4a_playback_get_next_capture(playback, &capture);
        if (stream_result != K4A_STREAM_RESULT_SUCCEEDED || capture == NULL)
        {
            if (stream_result == K4A_STREAM_RESULT_EOF) {
                return_code = 0;
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

        color_timestamp = k4a_image_get_device_timestamp_usec(color_image);
        sprintf (color_filename, "%s/color/%012ld.jpg", output_path, color_timestamp);
        depth_timestamp = k4a_image_get_device_timestamp_usec(depth_image);
        sprintf (depth_filename, "%s/depth/%012ld.png", output_path, depth_timestamp);

        if (K4A_RESULT_SUCCEEDED !=
            k4a_transformation_depth_image_to_color_camera(transformation, depth_image, transformed_depth_image))
        {
            printf("Failed to compute transformed depth image\n");
            return return_code;
        }

        buffer = k4a_image_get_buffer(color_image);
        size = k4a_image_get_size(color_image);
        
        img_array = cv::imdecode(cv::Mat(1, size, CV_8UC1, buffer), cv::IMREAD_UNCHANGED);
        cv::undistort(img_array, img_array_undistorted, matrix, distortion);
        cv::imwrite(color_filename, img_array_undistorted, compression_params);

        buffer = k4a_image_get_buffer(transformed_depth_image);
        size = k4a_image_get_size(transformed_depth_image);
        
        img_array = cv::Mat(color_image_height_pixels, color_image_width_pixels, CV_16UC1, buffer);
        cv::undistort(img_array, img_array_undistorted, matrix, distortion);
        cv::imwrite(depth_filename,  img_array_undistorted);

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

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    float res = static_cast< float >(i) * 1000000.0 / static_cast< float >(std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count());
    printf("%f fps\n", res);


    if (transformed_depth_image != NULL)
    {
        k4a_image_release(transformed_depth_image);
    }
    if (playback != NULL)
    {
        k4a_playback_close(playback);
    }
    if (transformation != NULL)
    {
        k4a_transformation_destroy(transformation);
    }

    return return_code;
}

int main(int argc, char **argv)
{
    int return_code = 0;

    if (argc != 3)
    {
        printf("Usage: mrob_images_extractor input.mkv output_path\n");
    }
    else
    {
        return_code = playback(argv[1], argv[2]);
    }

    return return_code;
}
