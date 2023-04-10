// Component
#include "rrt_sharp/py/RRTSharpPyInterface.hpp"

// Library
#include <boost/python.hpp>
#include <rrt_sharp/RRTSharp.hpp>

// Standard
#include <string>

void RRTSharpPyInterface::wrap()
{
    namespace bp = boost::python;

     bp::class_<RRTSharp>("RRTSharp")
             .def(boost::python::init<>())
             .def(boost::python::init<const std::vector<double>&>())
             .def("set_map_info", &RRTSharp::setMapInfo)
             .def("set_object_map", &RRTSharp::setObjectMap)
             .def("set_observed_map", &RRTSharp::setObservedMap)
             .def("set_terrain_map", &RRTSharp::setTerrainMap)
             .def("set_start_state", &RRTSharp::setStartState)
             .def("set_goal_state", &RRTSharp::setGoalState)
             .def("set_stop_radius", &RRTSharp::setStopRadius)
             .def("init", &RRTSharp::init)
             .def("run", &RRTSharp::run)

//             .add_property("resolution_mm", &RRTSharp::getResolutionMM, &RRTSharp::setResolutionMM)
//             .add_property("x_min_mm", &RRTSharp::getXMinMM, &RRTSharp::setXMinMM)
//             .add_property("x_max_mm", &RRTSharp::getXMaxMM, &RRTSharp::setXMaxMM)
//             .add_property("y_min_mm", &RRTSharp::getYMinMM, &RRTSharp::setYMinMM)
//             .add_property("y_max_mm", &RRTSharp::getYMaxMM, &RRTSharp::setYMaxMM)
//             .def("x_size", &RRTSharp::getXSize)
//             .def("y_size", &RRTSharp::getYSize)
//             .def("total_size", &RRTSharp::getTotalSize)
             ;
}
