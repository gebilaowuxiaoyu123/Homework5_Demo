cmake_minimum_required(VERSION 3.10)
project(BallisticCalculation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

# 查找依赖库
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
# 注释原ONNXRuntime的find_package（无法找到配置文件）
# find_package(ONNXRuntime REQUIRED)

# 包含所有模块的头文件目录
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/io
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/tasks
    ${CMAKE_CURRENT_SOURCE_DIR}/tools
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    # 替换为ONNXRuntime预编译包的头文件路径
    /usr/local/onnxruntime/include
)

# 添加ONNXRuntime的库文件目录（手动指定）
link_directories(/usr/local/onnxruntime/lib)

# 收集所有源文件
file(GLOB IO_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/io/*.cpp)
file(GLOB SRC_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB TASKS_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/tasks/*.cpp)
set(ALL_SRCS ${IO_SRCS} ${SRC_SRCS} ${TASKS_SRCS})

# 生成可执行文件
add_executable(ballistic_calc ${ALL_SRCS})

# 链接依赖库
target_link_libraries(ballistic_calc
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Boost_LIBRARIES}
    # 替换为ONNXRuntime的库名（预编译包中为onnxruntime）
    onnxruntime
    pthread  # 多线程支持
    fmt # 新增：链接fmt库
    onnxruntime #新增：链接ONNX Runtime库
)

# 安装配置（可选，将可执行文件安装到bin目录）
install(TARGETS ballistic_calc DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/bin)

