#ifndef POINT_HPP
#define POINT_HPP

/**
 * @brief The Point class
 */
class Point final
{
public:
    /**
     * @brief Point
     */
    Point();

    /**
     * @brief Point
     * @param x
     * @param y
     */
    Point(const double x, const double y);

    /**
     * @brief operator -
     * @param p
     * @return
     */
    Point operator-(const Point& p) const noexcept;

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
     * @brief distanceTo
     * @param p
     * @return
     */
    double distanceTo(const Point& p) const noexcept;

    /**
     * @brief setX
     * @param x
     */
    void setX(const double x) noexcept;

    /**
     * @brief setY
     * @param y
     */
    void setY(const double y) noexcept;

protected:
    double m_x; ///<
    double m_y; ///<
};

#endif // POINT_HPP
