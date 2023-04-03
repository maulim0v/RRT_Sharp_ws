#ifndef DUBINS_PARAMETERS_HPP
#define DUBINS_PARAMETERS_HPP

// Standard
#include <cmath>

namespace dubins
{

/**
 * The Parameters class
 */
class Parameters final
{
public:
    Parameters(const double alpha, const double beta, const double d) :
        m_alpha{alpha},
        m_beta{beta},
        m_d{d},
        m_sinAlpha{std::sin(alpha)},
        m_cosAlpha{std::cos(alpha)},
        m_sinBeta{std::sin(beta)},
        m_cosBeta{std::cos(beta)}
    {}

    double m_alpha{0.0}; ///< Angle difference between start state theta and angle between start/end states
    double m_beta{0.0};  ///< Angle difference between end state theta and angle between start/end states
    double m_d{0.0}; ///< Ratio between start/end state distance and rho (1/R [turning radius])
    double m_sinAlpha{0.0}; ///< Sine of alpha
    double m_cosAlpha{1.0}; ///< Cosine of alpha
    double m_sinBeta{0.0}; ///< Sine of beta
    double m_cosBeta{1.0}; ///< Cosine of beta
};

} // namespace dubins

#endif //DUBINS_PARAMETERS_HPP
