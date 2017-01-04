// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2016 Intel Corporation. All Rights Reserved.

#include <memory>
#include <iostream>
#include <fstream>
#include <json/json.h>
#include <librealsense/rs.hpp>

#include "rs_sdk.h"

using namespace rs::core;
using namespace std;

void enable_motion_tracking(rs::device * device)
{
    auto motion_callback = [](rs::motion_data motion_data)
    {
        //process motion data here
    };

    device->enable_motion_tracking(motion_callback);

    //set the camera to produce all streams timestamps from a single clock - the microcontroller's clock.
    //this option takes effect only if motion tracking is enabled and device->start() is called with rs::source::all_sources argument.
    device->set_option(rs::option::fisheye_strobe, 1);
}

rs::format string_to_format(const std::string& format_string)
{
    if (format_string == "any")
        return rs::format::any;
    if (format_string == "z16")
        return rs::format::z16;
    if (format_string == "disparity16")
        return rs::format::disparity16;
    if (format_string == "xyz32f")
        return rs::format::xyz32f;
    if (format_string == "yuyv")
        return rs::format::yuyv;
    if (format_string == "rgb8")
        return rs::format::rgb8;
    if (format_string == "bgr8")
        return rs::format::bgr8;
    if (format_string == "rgba8")
        return rs::format::rgba8;
    if (format_string == "bgra8")
        return rs::format::bgra8;
    if (format_string == "y8")
        return rs::format::y8;
    if (format_string == "y16")
        return rs::format::y16;
    if (format_string == "raw10")
        return rs::format::raw10;
    if (format_string == "raw16")
        return rs::format::raw16;
    if (format_string == "raw8")
        return rs::format::raw8;

    return rs::format::any;
}

int main(int argc, char* argv[]) try
{
    if (argc < 3)
    {
        cerr << "missing record file and camera config argument" << endl;
        return -1;
    }

    const string output_file(argv[1]);
    std::ifstream config(argv[2], std::ifstream::binary);

    Json::Value root;
    Json::Reader reader;
    if (!reader.parse(config, root))
    {
        cerr  << "Failed to parse configuration\n"
              << reader.getFormattedErrorMessages();
        return -1;
    }

    const int number_of_frames = root.get("frame_counts", 200).asInt();
    const Json::Value color = root["color"];
    const Json::Value depth = root["depth"];
    const Json::Value fisheye = root["fisheye"];

    //create a record enabled context with a given output file
    rs::record::context context(output_file.c_str());

    if(context.get_device_count() == 0)
    {
        cerr<<"no device detected" << endl;
        return -1;
    }

    //each device created from the record enabled context will write the streaming data to the given file
    rs::device* device = context.get_device(0);

    //enable required streams
    device->enable_stream(rs::stream::color,
                          color.get("width", 640).asInt(),
                          color.get("height", 480).asInt(),
                          string_to_format(color.get("format", "rgba8").asString()),
                          color.get("framerate", 30).asInt());
    device->enable_stream(rs::stream::depth,
                          depth.get("width", 640).asInt(),
                          depth.get("height", 480).asInt(),
                          string_to_format(depth.get("format", "z16").asString()),
                          depth.get("framerate", 30).asInt());
    vector<rs::stream> streams = { rs::stream::color, rs::stream::depth };

    if (!fisheye.isNull())
    {
        device->enable_stream(rs::stream::fisheye,
                              fisheye.get("width", 640).asInt(),
                              fisheye.get("height", 480).asInt(),
                              string_to_format(fisheye.get("format", "raw8").asString()),
                              fisheye.get("framerate", 30).asInt());
        streams.push_back(rs::stream::fisheye);
    }

    for(auto stream : streams)
    {
        std::cout << "stream type: " << stream << ", width: " << device->get_stream_width(stream) << ", height: " << device->get_stream_height(stream) << ", format: " << device->get_stream_format(stream) << ", fps: " << device->get_stream_framerate(stream) << std::endl;
    }

    //enable motion tracking, provides IMU events, mandatory for fisheye stream timestamp sync.
    enable_motion_tracking(device);

    device->start(rs::source::all_sources);

    for(auto i = 0; i < number_of_frames; ++i)
    {
        //each available frame will be written to the output file
        device->wait_for_frames();
        for(auto stream : streams)
        {
            if(device->is_stream_enabled(stream))
                std::cout << "stream type: " << stream << ", timestamp: " << device->get_frame_timestamp(stream) << std::endl;
        }
    }
    device->stop(rs::source::all_sources);

    return 0;
}

catch(rs::error e)
{
    std::cout << e.what() << std::endl;
    return -1;
}
