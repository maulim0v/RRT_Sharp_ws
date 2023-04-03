#ifndef KEY_COMPARE_HPP
#define KEY_COMPARE_HPP

// Component
#include "Key.hpp"

// Standard
#include <utility>

/**
 *
 */
class KeyCompare
{
public:
    /**
     * Key comparison functor
     * @param keyLeft Left key (f/g) to compare with
     * @param keyRight Right key (f/g) to compare with
     *
     * @return True if left key > right key
     */
    bool operator()(const Key& keyLeft, const Key& keyRight) const noexcept;
};

#endif // KEY_COMPARE_HPP
