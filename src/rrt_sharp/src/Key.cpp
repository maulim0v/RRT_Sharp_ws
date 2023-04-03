// Component
#include "Key.hpp"

// Standard
#include <utility>

Key::Key(const double f, const double g) :
    m_values{f, g}
{}

const std::pair<double, double>& Key::get() const noexcept
{
    return m_values;
}
