#ifndef DUBINS_MANEUVER_HPP
#define DUBINS_MANEUVER_HPP

// Component
#include "dubins/Direction.hpp"

// Standard
#include <array>

namespace dubins
{

/**
 * The Maneuver class
 */
class Maneuver final
{
public:
    /**
     * Default constructor
     *
     * @param t Path length of the first segment
     * @param p Path length of the second segment
     * @param q Path length of the third segment
     * @param length Total path length
     * @param combination Path combination
     */
    Maneuver(const double t,
             const double p,
             const double q,
             const double length,
             const std::array<Direction, 3>& combination) :
        m_t{t},
        m_p{p},
        m_q{q},
        m_length{length},
        m_combination{combination}
    {}

    double m_t; ///< Path length of the first segment
    double m_p; ///< Path length of the second segment
    double m_q; ///< Path length of the third segment
    double m_length; ///< Total path length
    std::array<Direction, 3> m_combination; ///< Segments combination
};

} // namespace dubins

#endif //DUBINS_MANEUVER_HPP
