// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include <stdio.h>
#include <malloc.h>
#include <k4a/k4a.h>
#include <k4arecord/playback.h>

#define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

#define MAX_IMU_SAMPLE_NUMBER (1600 * 3600 * 2) // ( frame rate * seconds in an hour * hours)

int main(int argc, char **argv)
{
    //printf("%d", argc);
    //printf(argv[1]);

    if (argc < 3) {
        printf("Usage: mrob_imu_data_extractor input.mkv output.csv\n");
        return 1;
    }

    k4a_playback_t playback_handle = NULL;
    if (k4a_playback_open(argv[1], &playback_handle) != K4A_RESULT_SUCCEEDED)
    {
        printf("Failed to open recording\n");
        return 1;
    }


    k4a_imu_sample_t imu_sample = { 0 };
    FILE *fpt;
    fpt = fopen(argv[2], "w+");
    uint64_t last_imu_timestamp = 0;

    //long long int recording_length = k4a_playback_get_last_timestamp_usec(playback_handle);
    //printf("Recording is %lld useconds long\n", recording_length);
    k4a_playback_seek_timestamp(playback_handle, 0, K4A_PLAYBACK_SEEK_END);
    k4a_playback_get_previous_imu_sample(playback_handle, &imu_sample);
    last_imu_timestamp = min(imu_sample.gyro_timestamp_usec, imu_sample.acc_timestamp_usec);
    k4a_playback_seek_timestamp(playback_handle, 0, K4A_PLAYBACK_SEEK_BEGIN);
    
    fprintf(fpt, "ot,ox,oy,oz,at,ax,ay,az\n");
    for (uint32_t i = 0; i < MAX_IMU_SAMPLE_NUMBER; i++) {
        k4a_playback_get_next_imu_sample(playback_handle, &imu_sample);
        fprintf(fpt, "%ld,%f,%f,%f,%ld,%f,%f,%f\n", \
            imu_sample.gyro_timestamp_usec, imu_sample.gyro_sample.v[0], imu_sample.gyro_sample.v[1], imu_sample.gyro_sample.v[2], \
            imu_sample.acc_timestamp_usec, imu_sample.acc_sample.v[0], imu_sample.acc_sample.v[1], imu_sample.acc_sample.v[2]);
        if (max(imu_sample.gyro_timestamp_usec, imu_sample.acc_timestamp_usec) >= last_imu_timestamp) {
            break;
        }
    }    
    
    k4a_playback_close(playback_handle);
}
