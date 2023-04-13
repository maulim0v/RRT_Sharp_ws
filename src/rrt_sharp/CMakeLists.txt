cmake_minimum_required(VERSION 3.0.2)
project(rrt_sharp)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall  -O3 -fPIC" )

#find_package(catkin REQUIRED COMPONENTS
#    roscpp
#)

#find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
#find_package(Python3 COMPONENTS NumPy)
find_package(ompl REQUIRED)
find_package(Eigen3)
find_package(Boost COMPONENTS python REQUIRED)
find_package(PythonLibs 3.8 REQUIRED)

#catkin_package(
#  INCLUDE_DIRS include
#)

#include_directories(
#    include
#    ${catkin_INCLUDE_DIRS}
#)

set(RRT_SHARP_SOURCES

    src/Point.cpp
    src/Vector.cpp
    src/Line.cpp
    src/State.cpp
    src/GoalState.cpp
    src/GraphNode.cpp
    src/Edge.cpp

    src/RRTSharp.cpp
    src/Cost.cpp
    src/Key.cpp
    src/KeyCompare.cpp
    src/Node.cpp
    src/Path.cpp

    src/dubins/DubinsCurve.cpp

    src/Ray.cpp
    src/NodeCompare.cpp

)

set(RRT_SHARP_HEADERS

#    include/rrt_sharp/matplotlibcpp.h


    include/rrt_sharp/Point.hpp
    include/rrt_sharp/Vector.hpp
    include/rrt_sharp/Line.hpp
    include/rrt_sharp/State.hpp
    include/rrt_sharp/GoalState.hpp
    include/rrt_sharp/GraphNode.hpp
    include/rrt_sharp/Edge.hpp

    include/rrt_sharp/RRTSharp.hpp
    include/rrt_sharp/Cost.hpp
    include/rrt_sharp/Key.hpp
    include/rrt_sharp/KeyCompare.hpp
    include/rrt_sharp/Node.hpp
    include/rrt_sharp/Path.hpp

    include/rrt_sharp/dubins/DubinsCurve.hpp
    include/rrt_sharp/dubins/Helper.hpp
    include/rrt_sharp/dubins/Parameters.hpp
    include/rrt_sharp/dubins/Combination.hpp
    include/rrt_sharp/dubins/Maneuver.hpp
    include/rrt_sharp/dubins/Direction.hpp

    include/rrt_sharp/Ray.hpp
    include/rrt_sharp/NodeCompare.hpp
)
add_library(rrt_sharp ${RRT_SHARP_SOURCES} ${RRT_SHARP_HEADERS})
target_include_directories(rrt_sharp PUBLIC #[[${catkin_INCLUDE_DIRS}]]
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/rrt_sharp>
  $<INSTALL_INTERFACE:include}>
)
target_link_libraries(rrt_sharp
#  Python3::Python
#  Python3::Module
#  Python3::NumPy
  ompl
)

#add_executable(rrt_sharp_exe src/RRTSharpNode.cpp)
#target_link_libraries(rrt_sharp_exe
#  rrt_sharp
#)

set(PY_SOURCES
    src/py/converter/IterableConverter.cpp
    src/py/main.cpp
    src/py/RRTSharpPyInterface.cpp
)

set(PY_HEADERS
    include/rrt_sharp/py/converter/IterableConverter.hpp
    include/rrt_sharp/py/RRTSharpPyInterface.hpp
)

set(CMAKE_SHARED_MODULE_PREFIX "")

add_library(
  rrt_sharp_py MODULE
    ${PY_SOURCES}
    ${PY_HEADERS}
)

target_link_libraries(
  rrt_sharp_py
    ${Boost_LIBRARIES}
    ${PYTHON_LIBRARIES}
    rrt_sharp
)

target_include_directories(
  rrt_sharp_py PRIVATE
    ${PYTHON_INCLUDE_DIRS}
    include
)
