// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2016 Intel Corporation. All Rights Reserved.

#include "rs_sdk.h"
#include "samples_time_sync_zr300.h"
#include "samples_time_sync_ds5.h"
#include "rs_sdk_version.h"

namespace rs {
    namespace utils {
        samples_time_sync_interface *
        samples_time_sync_interface::create_instance(
                int streams_fps[], int motions_fps[], const char* device_name, unsigned int max_input_latency,
                        unsigned int not_matched_frames_buffer_size)
        {
            if (device_name != nullptr)
            {
                std::string str(device_name);
                if ( str.find("ZR300") != std::string::npos )
                        return new samples_time_sync_zr300(streams_fps, motions_fps, max_input_latency, not_matched_frames_buffer_size);
                /*if ( str.find("RS400") != std::string::npos )
                        return new samples_time_sync_ds5(streams_fps, motions_fps, max_input_latency, not_matched_frames_buffer_size);*/

            }

            throw std::invalid_argument("Unsupported device or missing device name.");
        }

        samples_time_sync_interface *
        samples_time_sync_interface::create_instance(int streams_fps[], int motions_fps[], const char* device_name, unsigned int max_input_latency)
        {
            return samples_time_sync_interface::create_instance(streams_fps, motions_fps, device_name, max_input_latency, 0);
        }

        samples_time_sync_interface *
        samples_time_sync_interface::create_instance(int streams_fps[], int motions_fps[], const char* device_name)
        {
            return samples_time_sync_interface::create_instance(streams_fps, motions_fps, device_name, 100, 0);
        }
    }
}

