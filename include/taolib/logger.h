#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <functional>
#include <vector>

namespace tao {

class Logger {
public:
	enum class Level {
		DEBUG,
		INFO,
		WARNING,
		ERROR,
		FATAL
	};

	using Handle = std::function<void(Level, const std::string&)>;

	Logger(std::ostream& output_stream = std::cout, Level level = Level::INFO);
	
	void set_level(Level level);

	void add_handle(const Handle& handle);

	void debug(const char* format, ...) const;

	void info(const char* format, ...) const;

	void warning(const char* format, ...) const;

	void error(const char* format, ...) const;

	void fatal(const char* format, ...) const;

private:
	std::ostream& output_stream_;
	Level level_;
	std::vector<std::function<void(Level, const std::string&)>> handles_;
	
	void log(Level level, const char* format, va_list args) const;

	static std::string level_to_string(Level level);

	static std::string colorize(const std::string& message, Level level);

	static const std::string format(const char * const format, va_list args);
};

}