#ifndef PATH_HPP
#define PATH_HPP

// Component
#include "State.hpp"

// Standard
#include <iostream>
#include <vector>

/**
 *
 */
class Path final
{
public:
    /**
     *
     */
    Path();

    /**
     * Path
     *
     * @param states
     * @param length
     */
    Path(const std::vector<State>& states, const double length);

    /**
     * @brief states
     * @return
     */
    const std::vector<State>& states() const noexcept;

    /**
     * @brief length
     * @return
     */
    double length() const noexcept;

private:
    std::vector<State> m_states; ///<
    double m_length; ///<
};

#endif // PATH_HPP
