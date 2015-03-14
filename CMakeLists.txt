cmake_minimum_required(VERSION 2.8)
project(mp3_rrt)

set(CMAKE_CXX_FLAGS "-std=c++0x -march=native -mno-avx ${CMAKE_CXX_FLAGS}")

SET(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(Eigen REQUIRED)
find_package(Boost COMPONENTS system python thread program_options REQUIRED)
find_package(OpenRAVE REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp rospy)

include_directories(include)
include_directories(${OpenRAVE_INCLUDE_DIRS})
link_directories(${OpenRAVE_LIBRARY_DIRS})

catkin_package()

add_executable(rrt_plugin_exec src/rrt_plugin.cpp src/main.cpp)
set_target_properties(rrt_plugin_exec PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}" LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(rrt_plugin_exec ${catkin_LIBRARIES} ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES})

#add_library(rrt_plugin SHARED src/rrt_plugin.cpp)
#set_target_properties(rrt_plugin PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}" LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
#target_link_libraries(rrt_plugin ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES} ${Boost_LIBRARIES})

add_library(plugincpp SHARED src/plugin_test.cpp)
set_target_properties(plugincpp PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}" LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(plugincpp ${OpenRAVE_LIBRARIES})# ${OpenRAVE_CORE_LIBRARIES} ${Boost_LIBRARIES})

# 1427077258
# 1427077497

# Latest -
# 1427142869
