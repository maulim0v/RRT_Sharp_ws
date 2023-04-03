// Component
#include "GoalState.hpp"

GoalState::GoalState() :
//    State(),
    m_radius{0.0}
{}

GoalState::GoalState(const double x, const double y, const double z, const double radius, const double theta, const double rho) :
//    State(x, y, z, theta, rho),
    m_radius{radius}
{}

double GoalState::radius() const noexcept
{
    return m_radius;
}
