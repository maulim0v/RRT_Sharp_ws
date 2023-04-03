// Component
#include "Ray.hpp"

#include "Point.hpp"

// Standard
#include <cmath>
#include <cstdint>
#include <iostream>

Ray::Ray(const Point& start, const Point& end, const double resolution, const double minX, const double minY)
{
    const int startIndexX = static_cast<int>(std::floor((start.x() - minX) / resolution));
    const int startIndexY = static_cast<int>(std::floor((start.y() - minY) / resolution));

    const int endIndexX = static_cast<int>(std::floor((end.x() - minX) / resolution));
    const int endIndexY = static_cast<int>(std::floor((end.y() - minY) / resolution));

    generateRayIndeces(startIndexX, startIndexY, endIndexX, endIndexY);

    m_length = start.distanceTo(end);
}

void Ray::generateRayIndeces(const int startIndexX, const int startIndexY, const int endIndexX, const int endIndexY) noexcept
{
    const int dx = endIndexX - startIndexX;
    const int dy = endIndexY - startIndexY;

    const int dxPositive = std::abs(dx);
    const int dyPositive = std::abs(dy);

    int px = 2 * dyPositive - dxPositive;
    int py = 2 * dxPositive - dyPositive;

    int x = 0;
    int y = 0;
    int xStop = 0;
    int yStop = 0;

    if (dyPositive <= dxPositive)
    {
        if (dx >= 0)
        {
            x = startIndexX;
            y = startIndexY;
            xStop = endIndexX;
        }
        else
        {
            x = endIndexX;
            y = endIndexY;
            xStop = startIndexX;
        }

        m_indeces.emplace_back(x, y);

        for(int i = 0; x < xStop; ++i)
        {
            x = x + 1;

            if (px < 0)
            {
                px = px + 2 * dyPositive;
            }
            else
            {
                if(((dx < 0) && (dy < 0)) ||
                   ((dx > 0) && (dy > 0)))
                {
                    y = y + 1;
                }
                else
                {
                    y = y - 1;
                }

                px = px + 2 * (dyPositive - dxPositive);
            }

            m_indeces.emplace_back(x, y);
        }
    }
    else
    {
        if (dy >= 0)
        {
            x = startIndexX;
            y = startIndexY;
            yStop = endIndexY;
        }
        else
        {
            x = endIndexX;
            y = endIndexY;
            yStop = startIndexY;
        }

        m_indeces.emplace_back(x, y);

        for (int i = 0; y < yStop; ++i)
        {
            y = y + 1;

            if (py <= 0)
            {
                py = py + 2 * dxPositive;
            }
            else
            {
                if (((dx < 0) && (dy < 0)) ||
                    ((dx > 0) && (dy > 0)))
                {
                    x = x + 1;
                }
                else
                {
                    x = x - 1;
                }

                py = py + 2 * (dxPositive - dyPositive);
            }

            m_indeces.emplace_back(x, y);
        }
    }
}

const std::vector<std::pair<int, int>>& Ray::indeces() const noexcept
{
    return m_indeces;
}

double Ray::length() const noexcept
{
    return m_length;
}
