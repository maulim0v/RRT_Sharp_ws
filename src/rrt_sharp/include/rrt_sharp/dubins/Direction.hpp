#ifndef DUBINS_DIRECTION_HPP
#define DUBINS_DIRECTION_HPP

// Standard
#include <cstdint>

namespace dubins
{

/**
 * The Direction class
 */
enum class Direction : std::uint8_t
{
    NONE      = 0, ///< Direction unknown
    LEFT      = 1, ///< Direction Left
    STRAIGHT  = 2, ///< Direction Straight
    RIGHT     = 3, ///< Direction Right
};

} // namespace dubins

#endif // DUBINS_DIRECTION_HPP
