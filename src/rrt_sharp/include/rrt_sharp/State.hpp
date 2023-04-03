#ifndef STATE_HPP
#define STATE_HPP

// Component
#include "Point.hpp"

/**
 * @brief The State class
 */
class State final
{
public:
    /**
     * @brief State
     */
    State();

    /**
     * @brief State
     * @param p
     * @param theta
     * @param rho
     */
    State(const Point& point, const double theta);

    /**
     * @brief State
     * @param x
     * @param y
     * @param theta
     */
    State(const double x, const double y, const double theta);

    /**
     * @brief theta
     * @return
     */
    double theta() const noexcept;

    /**
     * @brief point
     * @return
     */
    const Point& point() const noexcept;

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

private:
    Point m_point; ///<
    double m_theta; ///<
};

#endif // STATE_HPP
