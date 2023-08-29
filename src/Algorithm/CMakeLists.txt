cmake_minimum_required(VERSION 3.8)
project(algorithm)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE -Ofast)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DPATH=\"\\\"${PROJECT_SOURCE_DIR}\\\"\"")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D${CMAKE_SYSTEM_NAME}")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(3RDPARTY_LIB "/home/wang/IRobot/RMOS/3rdparty/lib/")
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)


include_directories(
  ${EIGEN3_INCLUDE_DIR}
        ${3RDPARTY_LIB}
  /usr/include/eigen3

)
include_directories(include)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

# target_link_libraries(${PROJECT_NAME} Ceres::ceres ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS}   ${3RDPARTY_LIB}libonnxruntime.so  ${3RDPARTY_LIB}libonnxruntime.so.1.14.1)

#install(DIRECTORY
#        DESTINATION share/${PROJECT_NAME})


install(TARGETS algorithm
        DESTINATION lib/${PROJECT_NAME})
install(
        DIRECTORY  ${3RDPARTY_LIB}                   # 发现你项目中的lib中所有的文件
        DESTINATION lib/${PROJECT_NAME}      # 拷贝到install目录中
)
install(TARGETS algorithm
        RUNTIME DESTINATION lib/${PROJECT_NAME}    # 程序运行的时候调用install中的路径
        )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
)