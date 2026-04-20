#ifndef LOGGER_LOGGER_IMPL_H
#define LOGGER_LOGGER_IMPL_H

#include <array>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <utility>

#include "logger/logger.h"

// 抑制格式安全警告，因为format参数是来自用户可控的字符串字面量
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-security"

/**
 * @brief 字符串格式化辅助函数
 * @param format 格式化字符串
 * @param args 可变参数
 * @return 格式化后的字符串
 */
template <typename... Args>
inline std::string StringFormat(const char* format, Args&&... args)
{
    // 计算所需缓冲区大小
    int size = snprintf(nullptr, 0, format, std::forward<Args>(args)...);
    if (size <= 0)
    {
        return "";
    }

    // 分配缓冲区并格式化
    std::array<char, 1024> small_buf;
    if (size < static_cast<int>(small_buf.size()))
    {
        // 使用__attribute__((format(printf, 2, 3)))抑制格式安全警告
        // 因为format参数是来自用户可控的字符串字面量
        snprintf(small_buf.data(), small_buf.size(), format, std::forward<Args>(args)...);
        return small_buf.data();
    }
    else
    {
        std::string buf(size + 1, '\0');
        snprintf(buf.data(), buf.size(), format, std::forward<Args>(args)...);
        return buf;
    }
}

// 恢复诊断设置
#pragma GCC diagnostic pop

namespace logger
{

/**
 * @brief 日志记录模板函数实现
 */
template <typename... Args>
inline void Logger::Log(LogLevel level, const char* format, Args&&... args)
{
    // 检查日志级别
    if (level < GetInstance().log_level_)
    {
        return;
    }

    // 创建日志消息
    LogMessage msg;
    msg.level = level;
    msg.timestamp = std::chrono::system_clock::now();
    msg.thread_id = std::this_thread::get_id();
    msg.logger_name = GetInstance().name_;
    msg.message = ::StringFormat(format, std::forward<Args>(args)...);

    // 添加到队列
    {
        std::lock_guard<std::mutex> lock(GetInstance().queue_mutex_);
        GetInstance().log_queue_.push(std::move(msg));
    }

    // 通知写入线程
    GetInstance().queue_cv_.notify_one();
}

/**
 * @brief 带节流的日志记录模板函数实现
 */
template <typename... Args>
inline void Logger::LogThrottled(LogLevel level, uint64_t throttle_ms, const char* file, int line, const char* format,
                                 Args&&... args)
{
    // 检查日志级别
    if (level < GetInstance().log_level_)
    {
        return;
    }

    // 检查是否需要执行日志
    bool should_log = false;
    {
        std::lock_guard<std::mutex> lock(GetInstance().throttle_mutex_);

        // 创建日志唯一标识
        LogKey key;
        key.file = file;
        key.line = line;
        key.level = level;
        key.format = format;

        auto now = std::chrono::system_clock::now();
        auto it = GetInstance().throttle_map_.find(key);

        if (it == GetInstance().throttle_map_.end())
        {
            // 第一次执行该日志
            should_log = true;
            GetInstance().throttle_map_[key] = now;
        }
        else
        {
            // 检查时间差
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - it->second);
            if (duration.count() >= throttle_ms)
            {
                should_log = true;
                it->second = now;
            }
        }
    }

    // 如果需要执行日志
    if (should_log)
    {
        Log(level, format, std::forward<Args>(args)...);
    }
}

}  // namespace logger

#endif  // LOGGER_LOGGER_IMPL_H
