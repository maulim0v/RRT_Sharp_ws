#ifndef DUBINS_COMBINATION_HPP
#define DUBINS_COMBINATION_HPP

// Component
#include "dubins/Direction.hpp"

// Standard
#include <cstdint>

namespace dubins
{

/**
 * The Combination class
 */
enum class Combination : std::uint8_t
{
    NONE = 0, ///< Unknown combination
    RSR  = 1, ///< Right -> Straight -> Right
    RSL  = 2, ///< Right -> Straight -> Left
    LSR  = 3, ///< Left  -> Straight -> Right
    LSL  = 4, ///< Left  -> Straight -> Left
    RLR  = 5, ///< Right -> Left     -> Right
    LRL  = 6, ///< Left  -> Right    -> Left
};

inline Direction getDirection(const Combination combination, const int index)
{
    switch (combination)
    {
    case Combination::RSR:
        switch (index)
        {
        case 0:
            return Direction::RIGHT;
        case 1:
            return Direction::STRAIGHT;
        case 2:
            return Direction::RIGHT;
        default:
            break;
        }
    case Combination::RSL:
        switch (index)
        {
        case 0:
            return Direction::RIGHT;
        case 1:
            return Direction::STRAIGHT;
        case 2:
            return Direction::LEFT;
        default:
            break;
        }
    case Combination::LSR:
        switch (index)
        {
        case 0:
            return Direction::LEFT;
        case 1:
            return Direction::STRAIGHT;
        case 2:
            return Direction::RIGHT;
        default:
            break;
        }
    case Combination::LSL:
        switch (index)
        {
        case 0:
            return Direction::LEFT;
        case 1:
            return Direction::STRAIGHT;
        case 2:
            return Direction::LEFT;
        default:
            break;
        }
    case Combination::RLR:
        switch (index)
        {
        case 0:
            return Direction::RIGHT;
        case 1:
            return Direction::LEFT;
        case 2:
            return Direction::RIGHT;
        default:
            break;
        }
    case Combination::LRL:
        switch (index)
        {
        case 0:
            return Direction::LEFT;
        case 1:
            return Direction::RIGHT;
        case 2:
            return Direction::LEFT;
        default:
            break;
        }
    default:
        break;
    }

    return Direction::NONE;
}

} // namespace dubins

#endif // DUBINS_COMBINATION_HPP
