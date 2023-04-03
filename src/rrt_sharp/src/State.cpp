// Component
#include "State.hpp"

#include "Point.hpp"

State::State() :
    m_theta{0.0}
{}

State::State(const Point& point, const double theta) :
    m_point{point},
    m_theta{theta}
{}

State::State(const double x, const double y, const double theta) :
    m_point{x, y},
    m_theta{theta}
{}

const Point& State::point() const noexcept
{
    return m_point;
}

double State::theta() const noexcept
{
    return m_theta;
}

void State::setX(const double x) noexcept
{
    m_point.setX(x);
}

void State::setY(const double y) noexcept
{
    m_point.setY(y);
}
