#ifndef RAY_HPP
#define RAY_HPP

// Component
#include "Point.hpp"

// Standard
//#include <cmath>
#include <vector>

/**
 * The Ray class
 */
class Ray final
{
public:
    /**
     * Ray
     *
     * @param start Starting state for the ray
     * @param end Ending state for the ray
     * @param resolution Map resolution
     */
    Ray(const Point& start, const Point& end, const double resolution, const double minX, const double minY);

    /**
     * Returns ray indeces
     *
     * @return Ray indeces
     */
    const std::vector<std::pair<int, int>>& indeces() const noexcept;

    /**
     * @brief length
     * @return
     */
    double length() const noexcept;

private:    
    /**
     * Generates ray indeces
     *
     * @param start Start point
     * @param end End point
     */
    void generateRayIndeces(const int startIndexX, const int startIndexY, const int endIndexX, const int endIndexY) noexcept;

    std::vector<std::pair<int, int>> m_indeces; ///< Cell indeces X and Y for ray
    double m_length; ///< Length
};

#endif // RAY_HPP
