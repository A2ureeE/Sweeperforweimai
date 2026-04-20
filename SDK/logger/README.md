# 高性能日志系统

## 1. 设计概述

本日志系统是一个高性能、异步、线程安全的日志系统，参照ROS2日志设计，适用于C++项目。

### 1.1 核心特性

- **异步日志写入**：主线程将日志放入队列，独立线程负责写入文件
- **多级别日志**：支持DEBUG、INFO、WARN、ERROR、FATAL五个级别
- **双端输出**：同时输出到终端和文件
- **终端颜色**：不同级别日志显示不同颜色，便于区分
- **日志节流**：支持按时间节流，防止日志爆炸
- **文件滚动**：按大小自动滚动日志文件（默认10MB）
- **线程安全**：支持多线程并发写入
- **易用API**：提供简洁的宏定义接口

### 1.2 架构设计

```
+----------------+      +----------------+      +----------------+
|  业务线程      |      |  日志队列       |      |  日志写入线程   |
|  (LOG_* 宏)    | ---> |  (线程安全)     | ---> |  (文件+终端输出)|  
+----------------+      +----------------+      +----------------+
```

## 2. 安装与集成

### 2.1 CMakeLists.txt配置

```cmake
# 添加依赖
find_package(logger REQUIRED)

# 链接库
ament_target_dependencies(your_target
  logger
  # 其他依赖
)
```

### 2.2 头文件包含

```cpp
#include "logger/logger.h"
```

## 3. 使用方法

### 3.1 初始化与关闭

```cpp
// 初始化日志系统
logger::Logger::Init("node_name", "./nodes_log");

// 业务逻辑...

// 关闭日志系统（可选，程序结束时会自动关闭）
logger::Logger::Shutdown();
```

### 3.2 日志宏使用

#### 3.2.1 基本日志

```cpp
LOG_DEBUG("Debug message: %d", value);
LOG_INFO("Info message: %d", value);
LOG_WARN("Warn message: %d", value);
LOG_ERROR("Error message: %d", value);
LOG_FATAL("Fatal message: %d", value);
```

#### 3.2.2 带节流的日志

```cpp
// 每2000毫秒最多输出一次
LOG_INFO_THROTTLE(2000, "Throttled info: %d", value);
LOG_WARN_THROTTLE(1000, "Throttled warn: %d", value);
```

### 3.3 日志级别控制

```cpp
// 设置日志级别
logger::Logger::SetLogLevel(logger::LogLevel::DEBUG);
logger::Logger::SetLogLevel(logger::LogLevel::INFO);
logger::Logger::SetLogLevel(logger::LogLevel::WARN);
logger::Logger::SetLogLevel(logger::LogLevel::ERROR);
logger::Logger::SetLogLevel(logger::LogLevel::FATAL);

// 获取当前日志级别
logger::LogLevel level = logger::Logger::GetLogLevel();
```

## 4. 配置参数

### 4.1 初始化参数

```cpp
/**
 * @param name 日志记录器名称
 * @param log_dir 日志文件存储目录，默认值为"./log"
 * @param max_file_size 单个日志文件最大大小（字节），默认值为10MB
 * @param flush_interval 日志刷新间隔（毫秒），默认值为1000ms
 */
static void Init(const std::string& name,
                const std::string& log_dir = "./log",
                size_t max_file_size = 10 * 1024 * 1024,
                size_t flush_interval = 1000);
```

## 5. 日志格式

```
[YYYY-MM-DD HH:MM:SS.ms] [LEVEL] [THREAD_ID] [NAME] message
```

示例：
```
[2026-01-19 16:00:00.123] [INFO] [123456789] [test_node] Hello, Logger!
```

## 6. 终端颜色输出

日志系统为不同级别的日志提供了不同的终端颜色，便于直观区分：

| 日志级别 | 颜色   | ANSI 代码 | 描述       |
|---------|--------|-----------|------------|
| DEBUG   | 蓝色   | \033[34m  | 调试信息   |
| INFO    | 无颜色 | 无        | 正常信息   |
| WARN    | 黄色   | \033[33m  | 警告信息   |
| ERROR   | 红色   | \033[31m  | 错误信息   |
| FATAL   | 紫色   | \033[35m  | 致命错误   |

**说明**：
- 颜色仅在终端输出中显示，日志文件中为纯文本
- 颜色输出使用ANSI转义序列，兼容大多数现代终端
- 颜色设置会自动重置，不会影响后续终端输出
- 可以通过修改代码禁用颜色输出（如需）

## 7. 性能优化

1. **异步写入**：避免主线程阻塞
2. **批量处理**：减少磁盘I/O次数
3. **日志节流**：防止日志爆炸
4. **线程安全队列**：高效的多线程通信
5. **合理的刷新间隔**：平衡实时性和性能

## 8. 最佳实践

### 8.1 日志级别使用建议

- **DEBUG**：开发调试信息，生产环境建议关闭
- **INFO**：正常运行状态信息
- **WARN**：警告信息，需要关注但不影响正常运行
- **ERROR**：错误信息，可能影响部分功能
- **FATAL**：致命错误，会导致程序退出

### 8.2 日志内容建议

- 包含足够的上下文信息
- 使用清晰的格式
- 避免敏感信息
- 合理使用节流功能

### 8.3 性能考虑

- 避免在高频调用的函数中使用详细日志
- 对频繁输出的日志使用节流功能
- 生产环境建议使用INFO或更高级别

## 9. 示例代码

```cpp
#include "logger/logger.h"
#include <thread>

void TestThread(int id) {
    for (int i = 0; i < 5; ++i) {
        LOG_INFO_THROTTLE(1000, "Thread %d: info message %d", id, i);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

int main() {
    // 初始化日志系统
    logger::Logger::Init("test_logger");
    
    // 设置日志级别
    logger::Logger::SetLogLevel(logger::LogLevel::INFO);
    
    // 测试不同级别日志
    LOG_INFO("Logger initialized");
    LOG_WARN("This is a warning");
    LOG_ERROR("This is an error");
    
    // 测试多线程日志
    std::thread t1(TestThread, 1);
    std::thread t2(TestThread, 2);
    
    t1.join();
    t2.join();
    
    LOG_INFO("Test completed");
    
    return 0;
}
```

## 10. 版本变更

### 10.1 v1.0.0 (2026-01-19)

- 初始版本
- 实现基本日志功能
- 支持五种日志级别
- 异步日志写入
- 日志文件滚动

### 10.2 v1.1.0 (2026-01-19)

- 增加日志节流功能
- 支持所有级别日志同时输出到终端和文件
- 优化日志格式
- 增加详细文档

### 10.3 v1.2.0 (2026-01-19)

- 增加终端颜色输出功能
- 不同级别日志显示不同颜色
- 完善文档说明

## 11. 联系方式

如有问题或建议，请联系开发团队。
