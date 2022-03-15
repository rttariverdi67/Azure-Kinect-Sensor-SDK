// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include <stdio.h>
#include <malloc.h>
#include <k4a/k4a.h>
#include <k4arecord/playback.h>

#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include <iostream>
 
using namespace rapidjson;

StringBuffer parse_calib_params(k4a_calibration_camera_t calib)//, const char* camera_type) 
{
    StringBuffer s;
    Writer<StringBuffer> writer(s);

    writer.StartObject();
        //writer.Key(camera_type);
        //writer.StartObject();
            // Intrinsic params
            writer.Key("intrinsics");
            writer.StartObject();
                writer.Key("type");
                writer.Uint(calib.intrinsics.type);
                writer.Key("parameter_count");
                writer.Uint(calib.intrinsics.parameter_count);
                writer.Key("parameters");
                writer.StartObject();
                    writer.Key("parameters_as_dict");
                    writer.StartObject();
                        auto param = calib.intrinsics.parameters.param;
                        writer.Key("cx"); writer.Double(param.cx);
                        writer.Key("cy"); writer.Double(param.cy);
                        writer.Key("fx"); writer.Double(param.fx);
                        writer.Key("fy"); writer.Double(param.fy);
                        writer.Key("k1"); writer.Double(param.k1);
                        writer.Key("k2"); writer.Double(param.k2);
                        writer.Key("k3"); writer.Double(param.k3);
                        writer.Key("k4"); writer.Double(param.k4);
                        writer.Key("k5"); writer.Double(param.k5);
                        writer.Key("k6"); writer.Double(param.k6);
                        writer.Key("codx"); writer.Double(param.codx);
                        writer.Key("cody"); writer.Double(param.cody);
                        writer.Key("p2"); writer.Double(param.p2);
                        writer.Key("p1"); writer.Double(param.p1);
                    writer.EndObject();
                    writer.Key("parameters_as_list");
                    writer.StartArray();
                    std::cout << std::endl;
                    for (unsigned i = 0; i < 14; i++) {
                        writer.Double(calib.intrinsics.parameters.v[i]);
                    }
                    writer.EndArray();
                writer.EndObject();
            writer.EndObject();
            // Extrinsic params
            writer.Key("extrinsics");
            writer.StartObject();
                writer.Key("rotation");
                writer.StartArray();
                for (unsigned i = 0; i < 9; i++) {
                    writer.Double(calib.extrinsics.rotation[i]);
                }
                writer.EndArray();
                writer.Key("translation");
                writer.StartArray();
                for (unsigned i = 0; i < 3; i++) {
                    writer.Double(calib.extrinsics.translation[i]);
                }
                writer.EndArray();
            writer.EndObject();
            // Miscellaneous
            writer.Key("resolution_width");
            writer.Uint(calib.resolution_width);
            writer.Key("resolution_height");
            writer.Uint(calib.resolution_height);
            writer.Key("metric_radius");
            writer.Double(calib.metric_radius);
        //writer.EndObject();
    writer.EndObject();
    return s;
}


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
        printf("Calib params extractor. Failed to open recording\n");
        return 1;
    }

    k4a_calibration_t calib;

    if (k4a_playback_get_calibration (playback_handle, &calib) != K4A_RESULT_SUCCEEDED) {
        printf("Calib params extractor. Failed to get calib params\n");
        return 1;
    }
    
    k4a_calibration_camera_t calib_depth = calib.depth_camera_calibration;
    k4a_calibration_camera_t calib_color = calib.color_camera_calibration;

    //printf("%f", calib_depth.extrinsics.rotation[0]);
    
    StringBuffer s_d = parse_calib_params(calib_depth);
    StringBuffer s_c = parse_calib_params(calib_color);
    
    StringBuffer s;
    Writer<StringBuffer> writer(s);
    s.ShrinkToFit();
    Reader reader;
    writer.StartObject();
        writer.Key("depth_camera");
        StringStream ss_d(s_d.GetString());
        reader.Parse<0>(ss_d, writer);

        writer.Key("color_camera");
        StringStream ss_c(s_c.GetString());
        reader.Parse<0>(ss_c, writer);
    writer.EndObject();
    
    std::cout << s_d.GetString() << std::endl;
    std::cout << s_c.GetString() << std::endl;
    std::cout << s.GetString() << std::endl;



    /*StringBuffer s;
    Writer<StringBuffer> writer(s);
    
    writer.StartObject();               // Between StartObject()/EndObject(), 
    writer.Key("hello");                // output a key,
    writer.String("world");             // follow by a value.
    writer.Key("t");
    writer.Bool(true);
    writer.Key("f");
    writer.Bool(false);
    writer.Key("n");
    writer.Null();
    writer.Key("i");
    writer.Uint(123);
    writer.Key("pi");
    writer.Double(3.1416);
    writer.Key("a");
    writer.StartArray();                // Between StartArray()/EndArray(),
    for (unsigned i = 0; i < 4; i++)
        writer.Uint(i);                 // all values are elements of the array.
    writer.EndArray();
    writer.EndObject();

    // {"hello":"world","t":true,"f":false,"n":null,"i":123,"pi":3.1416,"a":[0,1,2,3]}
    std::cout << s.GetString() << std::endl;*/


    k4a_playback_close(playback_handle);
}

/*
    // 1. Parse a JSON string into DOM.
    const char* json = "{\"project\":\"rapidjson\",\"stars\":10}";
    Document d;
    d.Parse(json);
 
    // 2. Modify it by DOM.
    Value& s_ = d["stars"];
    s_.SetInt(s_.GetInt() + 1);
    //d["stars"] = "kek";
    
    // 3. Stringify the DOM
    StringBuffer buffer;
    Writer<StringBuffer> writer_(buffer);
    d.Accept(writer_);
 
    // Output {"project":"rapidjson","stars":11}
    std::cout << buffer.GetString() << std::endl;

*/