// Component
#include "KeyCompare.hpp"

// Standard
#include <utility>

bool KeyCompare::operator()(const Key& keyLeft, const Key& keyRight) const noexcept
{
    const std::pair<double, double>& keyLeftPair = keyLeft.get();
    const std::pair<double, double>& keyRightPair = keyRight.get();

    // f (first) strictly less than -> true
    if (keyLeftPair.first < keyRightPair.first)
    {
       return true;
    }

    // f (first) strictly higher than -> false
    if (keyLeftPair.first > keyRightPair.first)
    {
        return false;
    }

    // f (fist) are equal
    // g (second) less than or equal to -> true
    if (keyLeftPair.second <= keyRightPair.second)
    {
        return true;
    }

    // Otherwise,
    // f (first) are equal
    // g (second) higher than -> false
    return false;
}
