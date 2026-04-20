#ifndef LOGGER_LOGGER_H
#define LOGGER_LOGGER_H

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#include "logger/log_level.h"

namespace logger
{

/**
 * @brief 日志消息结构
 */
struct LogMessage
{
    LogLevel level;                                   ///< 日志级别
    std::string message;                              ///< 日志内容
    std::chrono::system_clock::time_point timestamp;  ///< 日志时间戳
    std::thread::id thread_id;                        ///< 线程ID
    std::string logger_name;                          ///< 日志记录器名称
};

/**
 * @brief 日志记录器类
 * 采用单例模式，支持异步日志写入
 */
class Logger
{
   public:
    /**
     * @brief 初始化日志记录器
     * @param name 日志记录器名称
     * @param log_dir 日志文件存储目录，默认值为"./log"
     * @param max_file_size 单个日志文件最大大小（字节），默认值为10MB
     * @param flush_interval 日志刷新间隔（毫秒），默认值为1000ms
     */
    static void Init(const std::string& name, const std::string& log_dir = "./log",
                     size_t max_file_size = 10 * 1024 * 1024, size_t flush_interval = 1000);

    /**
     * @brief 关闭日志记录器
     */
    static void Shutdown();

    /**
     * @brief 设置日志级别
     * @param level 日志级别
     */
    static void SetLogLevel(LogLevel level);

    /**
     * @brief 获取当前日志级别
     * @return 当前日志级别
     */
    static LogLevel GetLogLevel();

    /**
     * @brief 添加日志消息到队列
     * @param level 日志级别
     * @param format 日志格式化字符串
     * @param args 日志参数
     */
    template <typename... Args>
    static void Log(LogLevel level, const char* format, Args&&... args);

    /**
     * @brief 添加带节流的日志消息到队列
     * @param level 日志级别
     * @param throttle_ms 节流时间（毫秒）
     * @param file 文件路径
     * @param line 行号
     * @param format 日志格式化字符串
     * @param args 日志参数
     */
    template <typename... Args>
    static void LogThrottled(LogLevel level, uint64_t throttle_ms, const char* file, int line, const char* format,
                             Args&&... args);

   private:
    /**
     * @brief 构造函数
     */
    Logger();

    /**
     * @brief 析构函数
     */
    ~Logger();

    /**
     * @brief 日志写入线程函数
     */
    void WriteLogThread();

    /**
     * @brief 初始化日志文件
     */
    void InitLogFile();

    /**
     * @brief 滚动日志文件
     */
    void RotateLogFile();

    /**
     * @brief 格式化日志消息
     * @param msg 日志消息
     * @return 格式化后的日志字符串
     */
    std::string FormatLogMessage(const LogMessage& msg);

    /**
     * @brief 获取单例实例
     * @return 日志记录器单例
     */
    static Logger& GetInstance();

   private:
    struct LogKey
    {
        std::string file;    ///< 文件路径
        int line;            ///< 行号
        LogLevel level;      ///< 日志级别
        std::string format;  ///< 日志格式化字符串

        bool operator==(const LogKey& other) const
        {
            return file == other.file && line == other.line && level == other.level && format == other.format;
        }

        // 自定义哈希函数支持
        struct Hash
        {
            std::size_t operator()(const LogKey& key) const
            {
                return std::hash<std::string>{}(key.file) ^ std::hash<int>{}(key.line) ^
                       std::hash<int>{}(static_cast<int>(key.level)) ^ std::hash<std::string>{}(key.format);
            }
        };
    };

    using LogThrottleMap = std::unordered_map<LogKey, std::chrono::system_clock::time_point, LogKey::Hash>;

    std::string name_;                 ///< 日志记录器名称
    std::string log_dir_;              ///< 日志文件目录
    size_t max_file_size_;             ///< 单个日志文件最大大小
    size_t flush_interval_;            ///< 日志刷新间隔（毫秒）
    std::atomic<LogLevel> log_level_;  ///< 当前日志级别

    std::queue<LogMessage> log_queue_;  ///< 日志消息队列
    std::mutex queue_mutex_;            ///< 队列互斥锁
    std::condition_variable queue_cv_;  ///< 队列条件变量
    std::atomic<bool> running_;         ///< 运行状态标记
    std::thread write_thread_;          ///< 日志写入线程

    FILE* log_file_;                                         ///< 当前日志文件指针
    std::mutex file_mutex_;                                  ///< 文件互斥锁
    std::chrono::system_clock::time_point last_flush_time_;  ///< 上次刷新时间

    // 节流日志相关
    LogThrottleMap throttle_map_;  ///< 存储每个日志语句的最后执行时间
    std::mutex throttle_mutex_;    ///< 节流日志互斥锁
};

// 日志宏定义
#define LOG_DEBUG(format, ...) logger::Logger::Log(logger::LogLevel::DEBUG, format, ##__VA_ARGS__)
#define LOG_INFO(format, ...) logger::Logger::Log(logger::LogLevel::INFO, format, ##__VA_ARGS__)
#define LOG_WARN(format, ...) logger::Logger::Log(logger::LogLevel::WARN, format, ##__VA_ARGS__)
#define LOG_ERROR(format, ...) logger::Logger::Log(logger::LogLevel::ERROR, format, ##__VA_ARGS__)
#define LOG_FATAL(format, ...) logger::Logger::Log(logger::LogLevel::FATAL, format, ##__VA_ARGS__)

// 带节流的日志宏定义（单位：毫秒）
#define LOG_DEBUG_THROTTLE(throttle_ms, format, ...) \
    logger::Logger::LogThrottled(logger::LogLevel::DEBUG, throttle_ms, __FILE__, __LINE__, format, ##__VA_ARGS__)
#define LOG_INFO_THROTTLE(throttle_ms, format, ...) \
    logger::Logger::LogThrottled(logger::LogLevel::INFO, throttle_ms, __FILE__, __LINE__, format, ##__VA_ARGS__)
#define LOG_WARN_THROTTLE(throttle_ms, format, ...) \
    logger::Logger::LogThrottled(logger::LogLevel::WARN, throttle_ms, __FILE__, __LINE__, format, ##__VA_ARGS__)
#define LOG_ERROR_THROTTLE(throttle_ms, format, ...) \
    logger::Logger::LogThrottled(logger::LogLevel::ERROR, throttle_ms, __FILE__, __LINE__, format, ##__VA_ARGS__)
#define LOG_FATAL_THROTTLE(throttle_ms, format, ...) \
    logger::Logger::LogThrottled(logger::LogLevel::FATAL, throttle_ms, __FILE__, __LINE__, format, ##__VA_ARGS__)

}  // namespace logger

// 模板函数实现
#include "logger/logger_impl.h"

#endif  // LOGGER_LOGGER_H
