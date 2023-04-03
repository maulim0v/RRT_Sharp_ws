#ifndef DUBINS_DUBINS_CURVE_HPP
#define DUBINS_DUBINS_CURVE_HPP

// Component
#include "dubins/Direction.hpp"
#include "dubins/Helper.hpp"
#include "dubins/Maneuver.hpp"
#include "dubins/Parameters.hpp"
#include "State.hpp"

#include <math.h>
#include <string>
#include <vector>

namespace dubins
{

/**
 * The DubinsCurve class
 */
class DubinsCurve final
{
public:
    /**
     * DubinsCurve
     *
     * @param start Starting state for the curve
     * @param end Ending state for the curve
     * @param radius Turning radius to follow
     */
    DubinsCurve(const State& start, const State& end, const double radius = 10.0);

    /**
     * Returns path based on dubins maneuver
     *
     * @param resolution Resolution of the path
     * @return Path state
     */
    std::vector<State> getPath(const double resolution = 0.1) const;

    /**
     * @brief isValid
     * @return
     */
    bool isValid() const noexcept;

    /**
     * @brief getCost
     * @return
     */
    double getCost() const noexcept;

private:    
    /**
     * Generates dubins maneuver if successful
     *
     * @param start Start state for dubins path
     * @param end End state for dubins path
     * @return True if successful
     */
    bool generateManeuver(const State& start, const State& end) noexcept;

    /**
     * Maneuver generators
     *
     * @param a Alpha (angle between start theta and gamma angle (angle from start to end state))
     * @param b Beta (angle between end theta and gamma angle (angle from start to end state))
     * @param d Distance over turning radius
     * @param sa
     * @param ca
     * @param sb
     * @param cb
     * @return
     */
    /// @{
    Maneuver lsl(const double a, const double b, const double d, const double sa, const double ca, const double sb, const double cb) const noexcept;
    Maneuver rsr(const double a, const double b, const double d, const double sa, const double ca, const double sb, const double cb) const noexcept;
    Maneuver lsr(const double a, const double b, const double d, const double sa, const double ca, const double sb, const double cb) const noexcept;
    Maneuver rsl(const double a, const double b, const double d, const double sa, const double ca, const double sb, const double cb) const noexcept;
    Maneuver rlr(const double a, const double b, const double d, const double sa, const double ca, const double sb, const double cb) const noexcept;
    Maneuver lrl(const double a, const double b, const double d, const double sa, const double ca, const double sb, const double cb) const noexcept;
    /// @}

    /**
     * Intermediate state at a certain offset segment
     *
     * @param offset Offset/length on the path
     * @return State
     */
    State getCoordinatesAt(const double offset) const;

    /**
     * Position on the path based on direction
     *
     * @param offset Offset/length on the path
     * @param start Initial state the path starts from
     * @param direction Direction the path curves to
     * @return State
     */
    State getPositionInSegment(const double offset, const State& start, const Direction direction) const noexcept;

    State m_start; ///< Start state
    State m_end;   ///< End state
    double m_radius;  ///< Radius [turning radius]

    bool m_isValid; ///< Is valid maneuver exists
    Maneuver m_maneuver; ///< Dubins maneuver
};

} // namespace dubins

#endif // DUBINS_DUBINS_CURVE_HPP
