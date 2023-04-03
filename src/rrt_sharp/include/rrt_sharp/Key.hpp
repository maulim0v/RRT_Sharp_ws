#ifndef KEY_HPP
#define KEY_HPP

// Standard
#include <utility>

/**
 *
 */
class Key final
{
public:
    /**
     * Construct
     *
     * @param f Value f
     * @param g Value g
     */
    Key(const double f, const double g);

    /**
     *
     *
     * @return Value f and g pair
     */
    const std::pair<double, double>& get() const noexcept;

private:
    std::pair<double, double> m_values; ///< Pair of value F (first) and G (second)
};

#endif // KEY_HPP
