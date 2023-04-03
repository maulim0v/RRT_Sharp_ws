#ifndef LINE_HPP
#define LINE_HPP

// Component
#include "Point.hpp"
#include "Vector.hpp"

/**
 * @brief The Line class
 */
class Line
{
public:
    /**
     * @brief Line
     */
    Line();

    /**
     * @brief Line
     * @param p1
     * @param p2
     */
    Line(const Point& p1, const Point& p2);

    /**
     * @brief p1
     * @return
     */
    Point p1() const noexcept;

    /**
     * @brief p2
     * @return
     */
    Point p2() const noexcept;

    /**
     * @brief length
     * @return
     */
    double length() const noexcept;

    /**
     * @brief tangent
     * @return
     */
    Vector tangent() const noexcept;
protected:
    Point m_p1; ///<
    Point m_p2; ///<
    double m_length;  ///<
    Vector m_tangent; ///<
};

#endif // LINE_HPP
