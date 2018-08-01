#pragma once

#include <map>
#include <memory>
#include <sstream>
#include <vector>

#include "prettyprint.hpp"
#include "spdlog/logger.h"
#include "spdlog/sinks/null_sink.h"


namespace generic_logger {
/**
 * Pulling the level_enum into our namespace
 */
using level = spdlog::level::level_enum;
static const std::map<std::string, level> level_string{{spdlog::level::level_names[level::debug], level::debug},
                                                       {spdlog::level::level_names[level::info], level::info},
                                                       {spdlog::level::level_names[level::warn], level::warn},
                                                       {spdlog::level::level_names[level::err], level::err}};

/**
 * Extend the spd_logger to make sink and name changes possible.
 * This can't be done inside the singleton
 */
class Logger : public spdlog::logger {
    // Specify that base class constructors will be inherited
    using spdlog::logger::logger;

public:
    void set_sink(spdlog::sink_ptr sink) {
        auto sink_vtr = std::vector<spdlog::sink_ptr>{sink};
        set_sinks(sink_vtr);
    };

    void set_sinks(std::vector<spdlog::sink_ptr>& sinks) {
        _sinks.swap(sinks);
    }

    void set_name(const std::string name) {
        _name = name;
    }
};

/**
 * Create a singleton, so that tools can change logging behaviour and sink for libraries
 */
class LoggerSingleton {
public:
    static Logger& getInstance() {
        static auto sink = std::make_shared<spdlog::sinks::null_sink_mt>();
        static Logger instance("generic_logger", sink); // Guaranteed to be destroyed.
        // Instantiated on first use.
        return instance;
    }

private:
    // You want to make sure to delete these methods,
    // otherwise you may accidentally get copies of your singleton appearing.
    LoggerSingleton(LoggerSingleton const&) = delete;

    void operator=(LoggerSingleton const&) = delete;
};

/**
 * Pulling the set methods directly into our namespace
 */
inline void set_sinks(std::vector<spdlog::sink_ptr>& sinks) {
    LoggerSingleton::getInstance().set_sinks(sinks);
}

inline void set_sink(spdlog::sink_ptr sink) {
    LoggerSingleton::getInstance().set_sink(sink);
}

inline void set_level(level verbosity_level_enum) {
    LoggerSingleton::getInstance().set_level(verbosity_level_enum);
}
inline void set_level(const std::string& verbosity_level_string) {
    set_level(level_string.at(verbosity_level_string));
}
inline std::string get_level() {
    return spdlog::level::to_str(LoggerSingleton::getInstance().level());
}

inline void set_name(const std::string name) {
    LoggerSingleton::getInstance().set_name(name);
}

inline Logger& logger() {
    return LoggerSingleton::getInstance();
}

template <int Type>
class LogStream {
public:
    LogStream() = default;
    LogStream(const LogStream&) = delete;
    LogStream& operator=(const LogStream&) = delete;

    static_assert(Type >= 0 && Type <= 3, "Invalid log type specified.");

    template <typename T>
    LogStream& operator<<(T param) {
        ss << param;
        return *this;
    }

    typedef std::basic_ostream<char, std::char_traits<char>> CoutType;
    typedef CoutType& (*StandardEndLine)(CoutType&);

    LogStream& operator<<(StandardEndLine manip) {
        manip(ss);

        return *this;
    }

    ~LogStream() {
        switch (Type) {
        case 0:
            generic_logger::LoggerSingleton::getInstance().debug(ss.str());
            break;
        case 1:
            generic_logger::LoggerSingleton::getInstance().info(ss.str());
            break;
        case 2:
            generic_logger::LoggerSingleton::getInstance().warn(ss.str());
            break;
        case 3:
            generic_logger::LoggerSingleton::getInstance().error(ss.str());
            break;
        default:
            assert(false);
        }
    }

private:
    std::stringstream ss;
};

} // namespace generic_logger

/**
 * Define specific macros for fast access.
 * Note logger->doSomething can still be used
 */
#define DEBUG_STREAM(args) generic_logger::LogStream<0>() << args
#define INFO_STREAM(args) generic_logger::LogStream<1>() << args
#define WARN_STREAM(args) generic_logger::LogStream<2>() << args
#define ERROR_STREAM(args) generic_logger::LogStream<3>() << args
