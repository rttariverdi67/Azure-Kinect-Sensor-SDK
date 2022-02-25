// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include <stdio.h>
#include <malloc.h>
#include <k4a/k4a.h>
#include <k4arecord/playback.h>

//#include <filesystem.hpp>
//#include <iostream>
//#include <experimental/filesystem>
//namespace fs = std::experimental::filesystem;

#include <iostream>
#include <string>
using namespace std;

typedef struct
{
    char *filename;
    k4a_playback_t handle;
    k4a_record_configuration_t record_config;
    k4a_capture_t capture;
} recording_t;

static uint64_t first_capture_timestamp(k4a_capture_t capture)
{
    uint64_t min_timestamp = (uint64_t)-1;
    k4a_image_t images[3];
    images[0] = k4a_capture_get_color_image(capture);
    images[1] = k4a_capture_get_depth_image(capture);
    images[2] = k4a_capture_get_ir_image(capture);

    for (int i = 0; i < 3; i++)
    {
        if (images[i] != NULL)
        {
            uint64_t timestamp = k4a_image_get_device_timestamp_usec(images[i]);
            if (timestamp < min_timestamp)
            {
                min_timestamp = timestamp;
            }
            k4a_image_release(images[i]);
            images[i] = NULL;
        }
    }

    return min_timestamp;
}

/*static void print_capture_info(int i, recording_t *file)
{
    k4a_image_t images[3];
    images[0] = k4a_capture_get_color_image(file->capture);
    images[1] = k4a_capture_get_depth_image(file->capture);
    images[2] = k4a_capture_get_ir_image(file->capture);
        
    printf("%d %-32s", i, file->filename);
    for (int i = 0; i < 3; i++)
    {
        if (images[i] != NULL)
        {
            uint64_t timestamp = k4a_image_get_device_timestamp_usec(images[i]);
            printf("  %7ju usec", timestamp);
            k4a_image_release(images[i]);
            images[i] = NULL;
        }
        else
        {
            printf("  %12s", "");
        }
    }
    printf("\n");
}*/



int main(int argc, char **argv)
{
    if (argc < 3)
    {
        printf("Usage: mrob_timestamps_extraction input.mkv output_path\n");
        return 1;
    }

    size_t i = 0;
    size_t file_count = (size_t)(argc - 1);
    k4a_result_t result = K4A_RESULT_SUCCEEDED;

    // Allocate memory to store the state of N recordings.
    recording_t *files = (recording_t *)malloc(sizeof(recording_t) * file_count);
    //printf("To allocate memory for playback (%zu bytes)\n", sizeof(recording_t) * file_count);
    if (files == NULL)
    {
        printf("Failed to allocate memory for playback (%zu bytes)\n", sizeof(recording_t) * file_count);
        return 1;
    }
    memset(files, 0, sizeof(recording_t) * file_count);

    // Open each recording file and validate they were recorded in master/subordinate mode.
    i = 0;

    files[i].filename = argv[i + 1];

    result = k4a_playback_open(files[i].filename, &files[i].handle);
    if (result != K4A_RESULT_SUCCEEDED)
    {
        printf("Failed to open file: %s\n", files[i].filename);
        return result == K4A_RESULT_SUCCEEDED ? 0 : 1;
    }

    // Read the first capture of each recording into memory.
    k4a_stream_result_t stream_result = k4a_playback_get_next_capture(files[i].handle, &files[i].capture);
    if (stream_result == K4A_STREAM_RESULT_EOF)
    {
        printf("ERROR: Recording file is empty: %s\n", files[i].filename);
        result = K4A_RESULT_FAILED;
        return result == K4A_RESULT_SUCCEEDED ? 0 : 1;
    }
    else if (stream_result == K4A_STREAM_RESULT_FAILED)
    {
        printf("ERROR: Failed to read first capture from file: %s\n", files[i].filename);
        result = K4A_RESULT_FAILED;
        return result == K4A_RESULT_SUCCEEDED ? 0 : 1;
    }

    if (result == K4A_RESULT_SUCCEEDED)
    {
        //printf("%-32s  %12s  %12s  %12s\n", "Source file", "COLOR", "DEPTH", "IR");
        //printf("==========================================================================\n");

        string output_path = string(argv[2]) + string("/color_timestamps.csv");
        FILE *fpt_color, *fpt_depth, *fpt_ir;
        fpt_color = fopen(output_path.c_str(), "w+");
        fprintf(fpt_color, "timestamp_us\n");

        output_path = string(argv[2]) + string("/depth_timestamps.csv");
        fpt_depth = fopen(output_path.c_str(), "w+");
        fprintf(fpt_depth, "timestamp_us\n");

        output_path = string(argv[2]) + string("/ir_timestamps.csv");
        fpt_ir = fopen(output_path.c_str(), "w+");
        fprintf(fpt_ir, "timestamp_us\n");


        int frame = 0;
        while(true)
        {
            frame++;
            uint64_t min_timestamp = (uint64_t)-1;
            recording_t *min_file = NULL;

            // Find the lowest timestamp out of each of the current captures.
            i = 0;
            if (files[i].capture != NULL)
            {
                uint64_t timestamp = first_capture_timestamp(files[i].capture);
                if (timestamp < min_timestamp)
                {
                    min_timestamp = timestamp;
                    min_file = &files[i];
                }
            }

            k4a_image_t image = k4a_capture_get_color_image(files[i].capture);
            if (image != NULL)
            {
                uint64_t timestamp = k4a_image_get_device_timestamp_usec(image);
                k4a_image_release(image);
                image = NULL;
                fprintf(fpt_color, "%ld\n", timestamp);
            }

            image = k4a_capture_get_depth_image(files[i].capture);
            if (image != NULL)
            {
                uint64_t timestamp = k4a_image_get_device_timestamp_usec(image);
                k4a_image_release(image);
                image = NULL;
                fprintf(fpt_depth, "%ld\n", timestamp);
            }
            
            image = k4a_capture_get_ir_image(files[i].capture);
            if (image != NULL)
            {
                uint64_t timestamp = k4a_image_get_device_timestamp_usec(image);
                k4a_image_release(image);
                image = NULL;
                fprintf(fpt_ir, "%ld\n", timestamp);
            }






            //print_capture_info(frame, min_file);
            

            k4a_capture_release(min_file->capture);
            min_file->capture = NULL;

            // Advance the recording with the lowest current timestamp forward.
            k4a_stream_result_t stream_result = k4a_playback_get_next_capture(min_file->handle, &min_file->capture);
            if (stream_result != K4A_STREAM_RESULT_SUCCEEDED)
            {
                if (stream_result == K4A_STREAM_RESULT_FAILED) {
                    printf("ERROR: Failed to read next capture from file: %s\n", min_file->filename);
                    //result = K4A_RESULT_FAILED;

                }
                break;
            }
        }
    }

    i = 0;
    if (files[i].handle != NULL)
    {
        k4a_playback_close(files[i].handle);
        files[i].handle = NULL;
    }

    free(files);
    return 0;//(result == K4A_RESULT_SUCCEEDED) || (K4A_STREAM_RESULT_EOF) ? 0 : 1;
}
