CMAKE_MINIMUM_REQUIRED(VERSION 3.12)

find_package(Threads REQUIRED)

# 收集UartIMU源文件
set(uartimu_src UartIMU/xuc_driver.cpp)
if(ENABLE_LEGACY_UART)
    list(APPEND uartimu_src UartIMU/uart_driver.cpp)
endif()

# 创建timed_serial库
ADD_LIBRARY(timedserial OBJECT timed_serial.cpp ${uartimu_src})

# 链接RMCVSerial库用于串口通信
if(ENABLE_LEGACY_UART)
    target_link_libraries(timedserial RMCVSerial)
endif()

# 设置C++标准
set_property(TARGET timedserial PROPERTY CXX_STANDARD 17)
