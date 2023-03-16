#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <ctime>
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

	Logger(std::ostream& output_stream = std::cout, Level level = Level::INFO)
		: output_stream_(output_stream), level_(level) {}

	void set_output_stream(std::ostream& output_stream) {
		output_stream_ = output_stream;
	}

	void set_level(Level level) {
		level_ = level;
	}

	void add_handle(Handle handle) {
		handles_.push_back(handle);
	}

	void log(const std::string& message, Level level) const {
		if (level >= level_) {
			std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
			output_stream_ << std::ctime(&now) << "[" << level_to_string(level) << "] " << message << std::endl;

			for (auto& handle : handles_) {
				handle(level, message);
			}
		}
	}

	void debug(const std::string& message) const {
		log(message, Level::DEBUG);
	}

	void info(const std::string& message) const {
		log(message, Level::INFO);
	}

	void warning(const std::string& message) const {
		log(message, Level::WARNING);
	}

	void error(const std::string& message) const {
		log(message, Level::ERROR);
	}
	
	void fatal(const std::string& message) const {
		log(message, Level::FATAL);
	}

private:
	std::ostream& output_stream_;
	Level level_;
	std::vector<std::function<void(Level, const std::string&)>> handles_;

	std::string level_to_string(Level level) const {
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
};

}