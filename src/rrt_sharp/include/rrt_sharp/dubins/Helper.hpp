#ifndef DUBINS_HELPER_HPP
#define DUBINS_HELPER_HPP

// Standard
#include <cmath>

namespace dubins
{

/**
 * The Helper class
 */
class Helper final
{
public:
    /**
     * Modulo by 2 pi
     *
     * @param val Value/angle to take the modulo of
     * @return Angle
     */
    static double mod2pi(const double val)
    {
        constexpr double modVal = 2.0 * M_PI;
        const int num = static_cast<int>(std::floor(val / modVal));
        return val - num * modVal;
    }
};

} // namespace dubins

#endif // DUBINS_HELPER_HPP
