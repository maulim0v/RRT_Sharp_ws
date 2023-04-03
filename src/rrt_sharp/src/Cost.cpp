// Component
#include "Cost.hpp"

Cost::Cost(const double value) :
    m_value{value}
{}

double Cost::value() const noexcept
{
    return m_value;
}
