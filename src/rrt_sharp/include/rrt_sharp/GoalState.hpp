#ifndef GOAL_STATE_HPP
#define GOAL_STATE_HPP

// Component
#include "State.hpp"

/**
 * @brief The GoalState class
 */
class GoalState final //: public State
{
public:
    /**
     * @brief GoalState
     */
    GoalState();

    /**
     * @brief GoalState
     * @param x
     * @param y
     * @param z
     * @param radius
     * @param theta
     * @param rho
     */
    GoalState(const double x, const double y, const double z, const double radius, const double theta, const double rho);

    /**
     * @brief radius
     * @return
     */
    double radius() const noexcept;
private:
    double m_radius; ///<
};

#endif // GOAL_STATE_HPP
