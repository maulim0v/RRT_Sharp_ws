#ifndef COST_HPP
#define COST_HPP

// Standard
#include <iostream>

/**
 * Definition of a cost value. Can represent the cost of a motion or the cost of a state.
 */
class Cost final
{
public:
    /**
     * Construct cost with a specified value
     *
     * @param value Value
     */
    explicit Cost(const double value = 0.0);

    /**
     * The value of the cost
     *
     * @return Value
     */
    double value() const noexcept;

private:
    double m_value; ///< The value of the cost
};

#endif // COST_HPP
