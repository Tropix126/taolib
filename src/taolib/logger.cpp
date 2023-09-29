#include "taolib/logger.h"

#include <iostream>
#include <fstream>
#include <string>
#include <stdexcept>
#include <functional>
#include <vector>
#include <cstdarg>
#include <cstdio>
#include <memory>

namespace tao {

Logger::Logger(std::ostream& output_stream, Level level)
		: output_stream_(output_stream), level_(level) {}

void Logger::set_level(Level level) { level_ = level; }
void Logger::add_handle(const Handle& handle) { handles_.push_back(handle); }

void Logger::log(Level level, const char* format, va_list args) const {
	if (level >= level_) {
		std::string message = this->format(format, args);
		va_end(args);

		output_stream_
			<< colorize("[" + level_to_string(level) + "] ", level)
			<< message
			<< colorize("\033[0m\n", level);

		for (auto& handle : handles_) {
			handle(level, message);
		}
	}
}

void Logger::debug(const char* format, ...) const {
	va_list args;
	va_start(args, format);
	log(Level::DEBUG, format, args);
	va_end(args);
}

void Logger::info(const char* format, ...) const {
	va_list args;
	va_start(args, format);
	log(Level::INFO, format, args);
	va_end(args);
}

void Logger::warning(const char* format, ...) const {
	va_list args;
	va_start(args, format);
	log(Level::WARNING, format, args);
	va_end(args);
}

void Logger::error(const char* format, ...) const {
	va_list args;
	va_start(args, format);
	log(Level::ERROR, format, args);
	va_end(args);
}

void Logger::fatal(const char* format, ...) const {
	va_list args;
	va_start(args, format);
	log(Level::FATAL, format, args);
	va_end(args);
}

std::string Logger::level_to_string(Level level) {
	switch (level) {
		case Level::DEBUG:
			return "DEBUG";
		case Level::INFO:
			return "INFO";
		case Level::WARNING:
			return "WARNING";
		case Level::ERROR:
			return "ERROR";
		case Level::FATAL:
			return "FATAL";
		default:
			return "";
	}
}

std::string Logger::colorize(const std::string& message, Level level) {
	switch (level) {
	case Level::DEBUG:
		return "\033[37m" + message; // white
	case Level::INFO:
		return "\033[36m" + message; // cyan
	case Level::WARNING:
		return "\033[33m" + message; // yellow
	case Level::ERROR:
		return "\033[31m" + message; // red
	case Level::FATAL:
		return "\033[41m" + message; // red background
	default:
		return message;
	}
}

const std::string Logger::format(const char * const format, va_list args) {
    // reliably acquire the size
    // from a copy of the variable argument array
    // and a functionally reliable call to mock the formatting
    va_list args_copy;
    va_copy(args_copy, args);
    const int len = vsnprintf(NULL, 0, format, args_copy);
    va_end(args_copy);

    // return a formatted string without risking memory mismanagement
    // and without assuming any compiler or platform specific behavior
    std::vector<char> zc(len + 1);
    vsnprintf(zc.data(), zc.size(), format, args);
    va_end(args);
    return std::string(zc.data(), len);
}

} // namespace tao
