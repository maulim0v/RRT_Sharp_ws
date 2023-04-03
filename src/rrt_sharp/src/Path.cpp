// Component
#include "Path.hpp"

Path::Path() :
    m_length{0.0}
{}

Path::Path(const std::vector<State>& states, const double length) :
    m_states{states},
    m_length{length}
{}

const std::vector<State>& Path::states() const noexcept
{
    return m_states;
}

double Path::length() const noexcept
{
    return m_length;
}
