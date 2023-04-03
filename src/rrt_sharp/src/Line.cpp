// Component
#include "Line.hpp"

#include "Point.hpp"
#include "Vector.hpp"

Line::Line() :
    m_length{0.0}
{}

Line::Line(const Point& p1, const Point& p2) :
    m_p1{p1},
    m_p2{p2},
    m_length{p1.distanceTo(p2)}/*,*/
//    m_tangent{(p2 - p1) / m_length}
{}

Point Line::p1() const noexcept
{
    return m_p1;
}

Point Line::p2() const noexcept
{
    return m_p2;
}

double Line::length() const noexcept
{
    return m_length;
}

Vector Line::tangent() const noexcept
{
    return m_tangent;
}
