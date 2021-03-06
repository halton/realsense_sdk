// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2016 Intel Corporation. All Rights Reserved.

#pragma once
#include "rs/utils/release_self_base.h"
#include "rs/core/release_interface.h"

namespace rs
{
    namespace utils
    {
        /**
         * @class self_releasing_array_data_releaser
         * @brief the self_releasing_array_data_releaser is an arrays deallocation memory management class.
         *
         * when release is called it will delete the array of data given in the constructor and itself using the
         * release_self_base class.
         */
        class self_releasing_array_data_releaser : public release_self_base<rs::core::release_interface>
        {
        public:
            /**
             * @brief self_releasing_array_data_releaser
             *
             * @param data  the allocated data pointer.
             */
            self_releasing_array_data_releaser(uint8_t* data) :data(data) {}

            /**
             * @brief release
             *
             * assumes the data provided needs to be release with operator delete []
             * @return int  number of instances
             */
            int release() const override
            {
                if(data)
                {
                    delete [] data;
                }
                return release_self_base::release(); //object destructed, return immediately
            }
        protected:
            ~self_releasing_array_data_releaser() {}
        private:
            uint8_t* data;
        };
    }
}
