#ifndef VECTOR_HPP
#define VECTOR_HPP

/**
 * @brief The Vector class
 */
class Vector
{
public:
    /**
     * @brief Vector
     */
    Vector();

    /**
     * @brief Vector
     * @param x
     * @param y
     * @param z
     */
    Vector(const double x, const double y, const double z);

    /**
     * @brief operator *
     * @param val
     * @return
     */
    Vector operator*(const double val) const noexcept;

    /**
     * @brief operator /
     * @param val
     * @return
     */
    Vector operator/(const double val) const noexcept;

    /**
     * @brief x
     * @return
     */
    double x() const noexcept;

    /**
     * @brief y
     * @return
     */
    double y() const noexcept;

    /**
     * @brief z
     * @return
     */
    double z() const noexcept;

    /**
     * @brief magnitude
     * @return
     */
    double magnitude() const noexcept;

    /**
     * @brief dot
     * @param v
     * @return
     */
    double dot(const Vector& v) const noexcept;

protected:
    double m_x; ///<
    double m_y; ///<
    double m_z; ///<
    double m_magnitude; ///<
};

#endif // VECTOR_HPP
