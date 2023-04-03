// Component
#include "Vector.hpp"

// Standard
#include <cmath>

Vector::Vector() :
    m_x{0.0},
    m_y{0.0},
    m_z{0.0},
    m_magnitude{0.0}
{}

Vector::Vector(const double x, const double y, const double z) :
    m_x{x},
    m_y{y},
    m_z{z},
    m_magnitude{std::sqrt(x * x + y * y + z * z)}
{}

Vector Vector::operator*(const double val) const noexcept
{
    return Vector(m_x * val, m_y * val, m_z * val);
}

Vector Vector::operator/(const double val) const noexcept
{
    return Vector(m_x / val, m_y / val, m_z / val);
}

double Vector::x() const noexcept
{
    return m_x;
}

double Vector::y() const noexcept
{
    return m_y;
}

double Vector::z() const noexcept
{
    return m_z;
}

double Vector::magnitude() const noexcept
{
    return m_magnitude;
}

double Vector::dot(const Vector& v) const noexcept
{
    return m_x * v.m_x + m_y * v.m_y + m_z * v.m_z;
}
