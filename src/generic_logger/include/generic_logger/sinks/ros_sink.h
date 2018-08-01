#pragma once

#include <memory>
#include <mutex>
#include <ostream>

#include <ros/console.h>
#include "../spdlog/sinks/sink.h"

namespace generic_logger {
using namespace spdlog;

namespace sinks {
using namespace spdlog::sinks;

class ros_sink : public sink {
public:
    void log(const details::log_msg& msg) override {
        switch (msg.level) {
        case level::trace:
        case level::debug:
            ROS_DEBUG_STREAM(msg.raw.str());
            break;
        case level::info:
            ROS_INFO_STREAM(msg.raw.str());
            break;
        case level::warn:
            ROS_WARN_STREAM(msg.raw.str());
            break;
        case level::err:
        case level::critical:
            ROS_ERROR_STREAM(msg.raw.str());
            break;
        default:
            /* do nothing */
            break;
        }
    }

    void flush() {
        std::cout << std::flush;
    }
};
}
}
