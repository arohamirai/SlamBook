cmake_minimum_required( VERSION 2.8 )
project( ch3 )

set(CMAKE_CXX_FLAGS "-std=c++11")

# 添加Eigen头文件
include_directories( "/usr/include/eigen3" )

# 添加Pangolin依赖
find_package( Pangolin )
include_directories( ${Pangolin_INCLUDE_DIRS} )

add_executable( test7 test7.cpp )
target_link_libraries( test7 ${Pangolin_LIBRARIES} )


