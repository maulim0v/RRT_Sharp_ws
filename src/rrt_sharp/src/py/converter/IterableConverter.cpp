// Component
#include "rrt_sharp/py/converter/IterableConverter.hpp"

// Library
#include <boost/python.hpp>

// Standard
#include <vector>

void IterableConverter::wrap()
{
    // Add more container convertions here
    // example:
    //              .fromPython(std::container<type>())
    //
    IterableConverter()
            .fromPython<std::vector<float>>()
            .fromPython<std::vector<double>>()
            .fromPython<std::vector<int>>()
            ;
}

template <typename Container>
IterableConverter& IterableConverter::fromPython()
{
    boost::python::converter::registry::push_back(&IterableConverter::convertible,
                                                  &IterableConverter::construct<Container>,
                                                  boost::python::type_id<Container>());

    return *this;
}

void* IterableConverter::convertible(PyObject *object)
{
    return PyObject_GetIter(object) ? object : NULL;
}

template <typename Container>
void IterableConverter::construct(PyObject* object, boost::python::converter::rvalue_from_python_stage1_data* data)
{
    // Object is a borrowed reference, so create a handle indicting it is
    // borrowed for proper reference counting
    bp::handle<> handle(bp::borrowed(object));

    // Obtain a handle to the memory block that the converter has allocated
    // for the C++ type.
    typedef bp::converter::rvalue_from_python_storage<Container> storage_type;
    void* storage = reinterpret_cast<storage_type*>(data)->storage.bytes;

    typedef bp::stl_input_iterator<typename Container::value_type> iterator;

    // Allocate the C++ type into the converter's memory block, and assign
    // its handle to the converter's convertible variable.  The C++
    // container is populated by passing the begin and end iterators of
    // the python object to the container's constructor
    new (storage) Container(
      iterator(bp::object(handle)), // begin
      iterator());                  // end
    data->convertible = storage;
}
