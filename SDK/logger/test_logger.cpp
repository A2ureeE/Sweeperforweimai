#include <chrono>
#include <thread>

#include "logger/logger.h"

void TestThread(int id)
{
    for (int i = 0; i < 5; ++i)
    {
        LOG_DEBUG("Thread %d: debug message %d", id, i);
        LOG_INFO("Thread %d: info message %d", id, i);
        LOG_WARN("Thread %d: warn message %d", id, i);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

int main()
{
    // 初始化日志系统
    logger::Logger::Init("test_logger", "./test_logs", 1 * 1024 * 1024);

    // 设置日志级别为DEBUG
    logger::Logger::SetLogLevel(logger::LogLevel::DEBUG);

    // 测试不同级别的日志
    LOG_DEBUG("Debug message");
    LOG_INFO("Info message");
    LOG_WARN("Warn message");
    LOG_ERROR("Error message");
    LOG_FATAL("Fatal message");

    // 测试带参数的日志
    LOG_INFO("Test with integers: %d, %d, %d", 1, 2, 3);
    LOG_INFO("Test with floats: %f, %f", 1.1, 2.2);
    LOG_INFO("Test with strings: %s, %s", "hello", "world");

    // 测试多线程日志
    std::thread t1(TestThread, 1);
    std::thread t2(TestThread, 2);
    std::thread t3(TestThread, 3);

    t1.join();
    t2.join();
    t3.join();

    // 测试日志文件滚动（生成大量日志）
    for (int i = 0; i < 2000; ++i)
    {
        LOG_INFO("Large log message %d: This is a test to generate large amount of log data to test log file rotation.",
                 i);
    }

    // 关闭日志系统
    logger::Logger::Shutdown();

    return 0;
}
