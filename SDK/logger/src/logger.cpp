#include "logger/logger.h"

#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <iostream>

namespace logger
{

/**
 * @brief 构造函数
 */
Logger::Logger()
    : name_(""),
      log_dir_("./log"),
      max_file_size_(10 * 1024 * 1024),
      flush_interval_(1000),
      log_level_(LogLevel::INFO),
      running_(false),
      log_file_(nullptr)
{
}

/**
 * @brief 析构函数
 */
Logger::~Logger() { Shutdown(); }

/**
 * @brief 初始化日志记录器
 */
void Logger::Init(const std::string& name, const std::string& log_dir, size_t max_file_size, size_t flush_interval)
{
    Logger& logger = GetInstance();
    logger.name_ = name;
    logger.log_dir_ = log_dir;
    logger.max_file_size_ = max_file_size;
    logger.flush_interval_ = flush_interval;

    // 创建日志目录
    std::filesystem::create_directories(logger.log_dir_);

    // 初始化日志文件
    logger.InitLogFile();

    // 启动日志写入线程
    logger.running_ = true;
    logger.write_thread_ = std::thread(&Logger::WriteLogThread, &logger);

    LOG_INFO("Logger initialized: name=%s, log_dir=%s, max_file_size=%zu, flush_interval=%zu", name.c_str(),
             log_dir.c_str(), max_file_size, flush_interval);
}

/**
 * @brief 关闭日志记录器
 */
void Logger::Shutdown()
{
    Logger& logger = GetInstance();
    if (logger.running_)
    {
        logger.running_ = false;
        logger.queue_cv_.notify_one();

        if (logger.write_thread_.joinable())
        {
            logger.write_thread_.join();
        }

        // 刷新并关闭日志文件
        if (logger.log_file_)
        {
            fflush(logger.log_file_);
            fclose(logger.log_file_);
            logger.log_file_ = nullptr;
        }

        LOG_INFO("Logger shutdown");
    }
}

/**
 * @brief 设置日志级别
 */
void Logger::SetLogLevel(LogLevel level)
{
    GetInstance().log_level_ = level;
    LOG_INFO("Log level set to %s", LogLevelToString(level).c_str());
}

/**
 * @brief 获取当前日志级别
 */
LogLevel Logger::GetLogLevel() { return GetInstance().log_level_; }

/**
 * @brief 初始化日志文件
 */
void Logger::InitLogFile()
{
    // 生成日志文件名
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    struct tm tm_info;
    localtime_r(&now_c, &tm_info);

    // 按节点名称和日期创建子目录
    char sub_dir[256];
    snprintf(sub_dir, sizeof(sub_dir), "%s/%s/%04d%02d%02d", log_dir_.c_str(), name_.c_str(),
             tm_info.tm_year + 1900, tm_info.tm_mon + 1, tm_info.tm_mday);
    
    // 创建子目录
    std::filesystem::create_directories(sub_dir);

    char filename[256];
    snprintf(filename, sizeof(filename), "%s/%s_%04d%02d%02d_%02d%02d%02d.log", sub_dir, name_.c_str(),
             tm_info.tm_year + 1900, tm_info.tm_mon + 1, tm_info.tm_mday, tm_info.tm_hour, tm_info.tm_min,
             tm_info.tm_sec);

    // 打开日志文件
    log_file_ = fopen(filename, "a");
    if (!log_file_)
    {
        std::cerr << "Failed to open log file: " << filename << std::endl;
        exit(EXIT_FAILURE);
    }

    last_flush_time_ = std::chrono::system_clock::now();

    LOG_INFO("Log file initialized: %s", filename);
}

/**
 * @brief 滚动日志文件
 */
void Logger::RotateLogFile()
{
    if (log_file_)
    {
        fflush(log_file_);
        fclose(log_file_);
        log_file_ = nullptr;
    }

    InitLogFile();
}

/**
 * @brief 格式化日志消息
 */
std::string Logger::FormatLogMessage(const LogMessage& msg)
{
    // 格式化时间
    auto now = msg.timestamp;
    auto now_c = std::chrono::system_clock::to_time_t(now);
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

    struct tm tm_info;
    localtime_r(&now_c, &tm_info);

    char time_buf[64];
    snprintf(time_buf, sizeof(time_buf), "%04d-%02d-%02d %02d:%02d:%02d.%03d", tm_info.tm_year + 1900,
             tm_info.tm_mon + 1, tm_info.tm_mday, tm_info.tm_hour, tm_info.tm_min, tm_info.tm_sec,
             static_cast<int>(now_ms.count()));

    // 格式化线程ID
    char thread_buf[32];
    snprintf(thread_buf, sizeof(thread_buf), "%lu", std::hash<std::thread::id>{}(msg.thread_id));

    // 组合日志消息
    char log_buf[2048];
    snprintf(log_buf, sizeof(log_buf), "[%s] [%s] [%s] [%s] %s\n", time_buf, LogLevelToString(msg.level).c_str(),
             thread_buf, msg.logger_name.c_str(), msg.message.c_str());

    return log_buf;
}

/**
 * @brief 日志写入线程函数
 */
void Logger::WriteLogThread()
{
    std::queue<LogMessage> local_queue;

    while (running_ || !log_queue_.empty())
    {
        {  // 从队列获取日志消息
            std::unique_lock<std::mutex> lock(queue_mutex_);

            // 等待新的日志消息或超时
            queue_cv_.wait_for(lock, std::chrono::milliseconds(flush_interval_),
                               [this]() { return !log_queue_.empty() || !running_; });

            // 将队列中的消息移动到本地队列，减少锁持有时间
            if (!log_queue_.empty())
            {
                local_queue.swap(log_queue_);
            }
        }

        // 写入本地队列中的所有日志
        while (!local_queue.empty())
        {
            LogMessage msg = std::move(local_queue.front());
            local_queue.pop();

            std::string formatted_msg = FormatLogMessage(msg);

            {  // 写入日志文件
                std::lock_guard<std::mutex> lock(file_mutex_);
                if (log_file_)
                {
                    fwrite(formatted_msg.c_str(), 1, formatted_msg.size(), log_file_);

                    // 检查文件大小是否需要滚动
                    if (ftell(log_file_) >= static_cast<long>(max_file_size_))
                    {
                        RotateLogFile();
                    }

                    // 定期刷新缓冲区
                    auto now = std::chrono::system_clock::now();
                    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_flush_time_) >=
                        std::chrono::milliseconds(flush_interval_))
                    {
                        fflush(log_file_);
                        last_flush_time_ = now;
                    }
                }
            }

            std::cerr << formatted_msg;
        }
    }

    // 最后刷新日志文件
    if (log_file_)
    {
        fflush(log_file_);
    }
}

/**
 * @brief 获取单例实例
 */
Logger& Logger::GetInstance()
{
    static Logger instance;
    return instance;
}

}  // namespace logger
