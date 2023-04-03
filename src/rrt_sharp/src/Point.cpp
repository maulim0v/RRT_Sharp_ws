// Component
#include "Point.hpp"

// Component
#include <cmath>

Point::Point() :
    m_x{0.0},
    m_y{0.0}
{}

Point::Point(const double x, const double y) :
    m_x{x},
    m_y{y}
{}

Point Point::operator-(const Point& p) const noexcept
{
    const double xDiff = m_x - p.m_x;
    const double yDiff = m_y - p.m_y;

    return Point(xDiff, yDiff);
}

double Point::x() const noexcept
{
    return m_x;
}

double Point::y() const noexcept
{
    return m_y;
}

double Point::distanceTo(const Point& p) const noexcept
{
    const double xVal = (m_x - p.m_x) * (m_x - p.m_x);
    const double yVal = (m_y - p.m_y) * (m_y - p.m_y);

    return std::sqrt(xVal + yVal);
}

void Point::setX(const double x) noexcept
{
    m_x = x;
}

void Point::setY(const double y) noexcept
{
    m_y = y;
}
