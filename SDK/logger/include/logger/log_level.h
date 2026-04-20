#ifndef LOGGER_LOG_LEVEL_H
#define LOGGER_LOG_LEVEL_H

#include <string>

namespace logger
{

/**
 * @brief 日志级别枚举
 */
enum class LogLevel
{
    DEBUG = 0,  ///< 调试信息
    INFO = 1,   ///< 普通信息
    WARN = 2,   ///< 警告信息
    ERROR = 3,  ///< 错误信息
    FATAL = 4   ///< 致命错误信息
};

/**
 * @brief 将日志级别转换为字符串
 * @param level 日志级别
 * @return 日志级别对应的字符串
 */
inline std::string LogLevelToString(LogLevel level)
{
    switch (level)
    {
        case LogLevel::DEBUG:
            return "DEBUG";
        case LogLevel::INFO:
            return "INFO";
        case LogLevel::WARN:
            return "WARN";
        case LogLevel::ERROR:
            return "ERROR";
        case LogLevel::FATAL:
            return "FATAL";
        default:
            return "UNKNOWN";
    }
}

/**
 * @brief 将字符串转换为日志级别
 * @param level_str 日志级别字符串
 * @return 对应的日志级别
 */
inline LogLevel StringToLogLevel(const std::string& level_str)
{
    if (level_str == "DEBUG")
    {
        return LogLevel::DEBUG;
    }
    else if (level_str == "INFO")
    {
        return LogLevel::INFO;
    }
    else if (level_str == "WARN")
    {
        return LogLevel::WARN;
    }
    else if (level_str == "ERROR")
    {
        return LogLevel::ERROR;
    }
    else if (level_str == "FATAL")
    {
        return LogLevel::FATAL;
    }
    else
    {
        return LogLevel::INFO;  // 默认日志级别
    }
}

}  // namespace logger

#endif  // LOGGER_LOG_LEVEL_H
