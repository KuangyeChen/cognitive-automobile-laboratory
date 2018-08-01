#include "generic_logger.hpp"

#include "gtest/gtest.h"

TEST(logger, set_sinks) {

	using namespace generic_logger;

	INFO_STREAM("Changing sink.");
// LoggerSingleton::getInstance().set_sinks(sink_type::STDOUT_SINK);
	INFO_STREAM("Sink changed.");
}

TEST(logger, stdout_sink) {

	DEBUG_STREAM("Debug" << std::endl);
	INFO_STREAM("Info" << std::endl);
	WARN_STREAM("Warn" << std::endl);
	ERROR_STREAM("Error" << std::endl);
}
