#include "taolib/utility/Logger.h"

#include <iostream>
#include <fstream>
#include <string>
#include <stdexcept>
#include <functional>
#include <vector>
#include <cstdarg>

namespace tao {

Logger::Logger(std::ostream& output_stream = std::cout, Level level = Level::INFO)
		: output_stream_(output_stream), level_(level) {}

void Logger::set_level(Level level) { level_ = level; }
void Logger::add_handle(const Handle& handle) { handles_.push_back(handle); }

void Logger::log(Level level, const char* format, ...) const {
	if (level >= level_) {
		va_list args;
		va_start(args, format);
		std::string message = this->format(format, args);
		va_end(args);

		output_stream_ << colorize("[" + level_to_string(level) + "] ", level) << message << colorize("\033[0m\n", level);

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

template<typename ... Args>
std::string Logger::format(const std::string& format, Args ... args) {
	int32_t size_s = std::snprintf(nullptr, 0, format.c_str(), args ...) + 1;

	if(size_s <= 0) {
		throw std::runtime_error( "Error during formatting." );
	}

	auto size = static_cast<size_t>(size_s);

	std::unique_ptr<char[]> buf(new char[ size ]);
	std::snprintf(buf.get(), size, format.c_str(), args ... );

	return std::string(buf.get(), buf.get() + size - 1);
}

} // namespace tao
