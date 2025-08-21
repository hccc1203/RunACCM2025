#pragma once

/**
 * @file logger.hpp
 * @brief Logger class for logging messages to a file.
 * @version 0.1
 * @date 2025-04-25
 */

#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>


/*
### `Logger` 类用法说明

#### 概述
`Logger` 类是一个用于将日志消息记录到文件的工具。它可以同时将日志输出到控制台和指定的文件中。日志消息会附带当前的时间戳。

#### 构造函数
- `Logger(const std::string &name, const std::string &filePath = "")`
  - `name`：日志记录器的名称。
  - `filePath`：可选参数，指定日志文件的路径。如果不提供该参数，日志将仅输出到控制台。

#### 析构函数
- `~Logger()`：在对象销毁时，如果有打开的日志文件，会关闭该文件。

#### 移动构造函数和移动赋值运算符
- `Logger(Logger &&) = default;`
- `Logger &operator=(Logger &&) = default;`
这两个函数允许 `Logger` 对象进行移动操作。

#### 重载的函数调用运算符
- `template <typename... Args> void operator()(Args... args)`
  - 可以接受任意数量和类型的参数，将这些参数组合成一条日志消息，并添加当前时间戳。
  - 日志消息会同时输出到控制台和指定的日志文件（如果有）。

#### 示例程序

##### 示例 1：仅输出到控制台
```cpp
#include "logger.hpp"

int main() {
    Logger logger("ConsoleLogger");
    logger("This is a console-only log message.");
    return 0;
}
```
这个示例创建了一个名为 `ConsoleLogger` 的日志记录器，由于没有指定文件路径，日志消息只会输出到控制台。

##### 示例 2：输出到文件和控制台
```cpp
#include "logger.hpp"

int main() {
    Logger logger("FileAndConsoleLogger", "example.log");
    logger("This log message will be saved to example.log and printed on the console.");
    return 0;
}
```
这个示例创建了一个名为 `FileAndConsoleLogger` 的日志记录器，并指定了日志文件路径为 `example.log`。日志消息会同时输出到控制台和 `example.log` 文件中。

##### 示例 3：记录多个参数
```cpp
#include "logger.hpp"
#include <string>

int main() {
    Logger logger("MultiParamLogger", "multi_param.log");
    int num = 42;
    std::string str = "Hello, Logger!";
    logger("The answer is", num, "and the message is", str);
    return 0;
}
```
这个示例展示了如何使用 `Logger` 记录多个不同类型的参数。日志消息会包含所有参数，并添加时间戳。

#### 注意事项
- 如果指定的文件路径无效或无法打开，日志将仅输出到控制台。
- 日志文件以追加模式打开，即每次程序运行时，新的日志消息会添加到文件末尾。
*/


class Logger {
  private:
    std::string name;
    std::ofstream logFile;
    bool hasFile;

    std::string getCurrentTime() {
        auto now = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);
        std::tm tm;
        localtime_r(&time, &tm);
        std::ostringstream oss;
        auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
        oss << "[" << "\033[32m" << std::put_time(&tm, "%Y-%m-%d %H:%M:%S") << "." 
            << std::setfill('0') << std::setw(3) << milliseconds.count() << "\033[0m"
            << "\033[36m " << name << "\033[0m" << "]";
        return oss.str();
    }

  public:
    Logger(const std::string &name, const std::string &filePath = "")
        : name(name), hasFile(false) {
        if (!filePath.empty()) {
            logFile.open(filePath, std::ios::app);
            hasFile = logFile.is_open();
        }
    }

    ~Logger() {
        if (hasFile) {
            logFile.close();
        }
    }

    // Support move semantics
    Logger(Logger &&) = default;
    Logger &operator=(Logger &&) = default;

    template <typename... Args> void operator()(Args... args) {
        std::stringstream ss;
        ss << getCurrentTime() << " ";
        ((ss << args << " "), ...);
        ss.seekp(-1, std::ios_base::end); // Remove the last space
        ss << std::endl;

        std::cout << ss.str();
        if (hasFile) {
            logFile << ss.str();
            logFile.flush();
        }
    }

    // Warning
    template <typename... Args> void warning(Args... args) {
        std::stringstream ss;
        ss << getCurrentTime() << " ";
        ss << "\033[33m";
        ((ss << args << " "), ...);
        ss.seekp(-1, std::ios_base::end); // Remove the last space
        ss << "\033[0m" << std::endl;

        std::cout << ss.str();
        if (hasFile) {
            logFile << ss.str();
            logFile.flush();
        }
    }

    // Error
    template <typename... Args> void error(Args... args) {
        std::stringstream ss;
        ss << getCurrentTime() << " ";
        ss << "\033[31m";
        ((ss << args << " "), ...);
        ss.seekp(-1, std::ios_base::end); // Remove the last space
        ss << "\033[0m" << std::endl;

        std::cout << ss.str();
        if (hasFile) {
            logFile << ss.str();
            logFile.flush();
        }
    }

    // Info
    template <typename... Args> void info(Args... args) {
        std::stringstream ss;
        ss << getCurrentTime() << " ";
        ss << "\033[34m";
        ((ss << args << " "), ...);
        ss.seekp(-1, std::ios_base::end); // Remove the last space
        ss << "\033[0m" << std::endl;

        std::cout << ss.str();
        if (hasFile) {
            logFile << ss.str();
            logFile.flush();
        }
    }
};