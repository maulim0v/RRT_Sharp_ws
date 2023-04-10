// Component
#include "rrt_sharp/py/converter/IterableConverter.hpp"
#include "rrt_sharp/py/RRTSharpPyInterface.hpp"

// Libnrary
#include <boost/python.hpp>

BOOST_PYTHON_MODULE(rrt_sharp_py)
{
    // Converter(s), add more container convertions into wrap
    IterableConverter::wrap();

    // RRT Sharp Py Interface
    RRTSharpPyInterface::wrap();
}
