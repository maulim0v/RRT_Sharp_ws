#ifndef ITERABLE_CONVERTER_HPP
#define ITERABLE_CONVERTER_HPP

// Library
#include <boost/python.hpp>

namespace bp = boost::python; ///< Boost python namespace

/// @brief Type that allows for registration of conversions from python iterable types
class IterableConverter final
{
public:
    /// @brief
    static void wrap();

private:
    /// @brief Registers converter from a python interable type to the provided type
    template <typename Container>
    IterableConverter& fromPython();

    /// @brief Check if PyObject is iterable
    /// @param object Python object
    static void* convertible(PyObject* object);

    /// @brief Convert iterable PyObject to C++ container type
    ///
    /// Container Concept requirements:
    ///
    ///   * Container::value_type is CopyConstructable
    ///   * Container can be constructed and populated with two iterators
    ///     I.e. Container(begin, end)
    template <typename Container>
    static void construct(PyObject* object, boost::python::converter::rvalue_from_python_stage1_data* data);
};

#endif // ITERABLE_CONVERTER_HPP
