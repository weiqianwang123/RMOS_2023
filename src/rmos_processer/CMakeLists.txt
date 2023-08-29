cmake_minimum_required(VERSION 3.8)
project(rmos_processer)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE -Ofast)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DPATH=\"\\\"${PROJECT_SOURCE_DIR}\\\"\"")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D${CMAKE_SYSTEM_NAME}")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

set(INCLUDES ${INCLUDES} $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
set(INCLUDES ${INCLUDES} $<INSTALL_INTERFACE:/include>)
set(3RDPARTY_LIB "/home/wang/IRobot/RMOS/3rdparty/lib/")
include_directories(include/${PROJECT_NAME})
# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)


include_directories(
  ${EIGEN3_INCLUDE_DIR}
  /usr/include/eigen3
        ${3RDPARTY_LIB}
)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_include_directories(${PROJECT_NAME}
                      PUBLIC ${OpenCV_INCLUDE_DIR}
                      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                        $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

install(TARGETS ${PROJECT_NAME}
       DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
        DESTINATION share/${PROJECT_NAME})

install(
        DIRECTORY  ${3RDPARTY_LIB}                   # 发现你项目中的lib中所有的文件
        DESTINATION lib/     # 拷贝到install目录中
)


rclcpp_components_register_node(${PROJECT_NAME}
        PLUGIN rmos_processer::ProcesserNode
        EXECUTABLE processer
        )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
)
