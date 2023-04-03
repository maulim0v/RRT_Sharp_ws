// Component
#include "dubins/DubinsCurve.hpp"

#include "dubins/Direction.hpp"
#include "dubins/Helper.hpp"
#include "Point.hpp"

// Standard
#include <cstdint>
#include <iostream>

namespace dubins
{

DubinsCurve::DubinsCurve(const State& start, const State& end, const double radius) :
    m_start{start},
    m_end{end},
    m_radius{radius},
    m_isValid{false},
    m_maneuver{INFINITY, INFINITY, INFINITY, INFINITY, {Direction::NONE, Direction::NONE, Direction::NONE}}
{
    m_isValid = generateManeuver(start, end);
}

bool DubinsCurve::generateManeuver(const State& start, const State& end) noexcept
{
    const Point diff   = end.point() - start.point();
    const double D     = start.point().distanceTo(end.point());
    const double gamma = dubins::Helper::mod2pi(std::atan2(diff.y(), diff.x()));
    const double alpha = dubins::Helper::mod2pi(start.theta() - gamma);
    const double beta  = dubins::Helper::mod2pi(end.theta()- gamma);
    const double d     = D / m_radius;

    std::vector<Maneuver> maneuvers;
    maneuvers.emplace_back(lsl(alpha, beta, d, std::sin(alpha), std::cos(alpha), std::sin(beta), std::cos(beta)));
    maneuvers.emplace_back(rsr(alpha, beta, d, std::sin(alpha), std::cos(alpha), std::sin(beta), std::cos(beta)));
    maneuvers.emplace_back(lsr(alpha, beta, d, std::sin(alpha), std::cos(alpha), std::sin(beta), std::cos(beta)));
    maneuvers.emplace_back(rsl(alpha, beta, d, std::sin(alpha), std::cos(alpha), std::sin(beta), std::cos(beta)));
    maneuvers.emplace_back(rlr(alpha, beta, d, std::sin(alpha), std::cos(alpha), std::sin(beta), std::cos(beta)));
    maneuvers.emplace_back(lrl(alpha, beta, d, std::sin(alpha), std::cos(alpha), std::sin(beta), std::cos(beta)));

    m_maneuver = Maneuver(INFINITY, INFINITY, INFINITY, INFINITY, {Direction::NONE, Direction::NONE, Direction::NONE});
    double shortestLength = INFINITY;
    for (const auto& maneuver : maneuvers)
    {
        if (maneuver.m_length < shortestLength)
        {
            m_maneuver = maneuver;
            shortestLength = maneuver.m_length;
        }
    }

    if (std::isfinite(shortestLength) == false)
    {
        return false;
    }

    return true;
}

std::vector<State> DubinsCurve::getPath(const double resolution) const
{
    const int numPoints = static_cast<int>(std::ceil(m_maneuver.m_length / resolution));
    std::vector<State> points(numPoints);

    for (int i = 0; i < numPoints; i++)
    {
        points[i] = getCoordinatesAt(i * resolution);
    }

    return points;
}

bool DubinsCurve::isValid() const noexcept
{
    return m_isValid;
}

double DubinsCurve::getCost() const noexcept
{
    static constexpr double curveScale = 1.5;

    double cost = (curveScale * (m_maneuver.m_t + m_maneuver.m_q) * m_radius);

    if ((m_maneuver.m_combination[1] == Direction::LEFT) ||
        (m_maneuver.m_combination[1] == Direction::RIGHT))
    {
        cost += (curveScale * m_maneuver.m_p * m_radius);
    }
    else
    {
        cost += (m_maneuver.m_p * m_radius);
    }

    return cost;
}

Maneuver DubinsCurve::lsl(const double a, const double b, const double d, const double sa, const double ca, const double sb, const double cb) const noexcept
{
    const double aux = std::atan2(cb - ca, d + sa - sb);

    const double t = dubins::Helper::mod2pi(-a + aux);
    const double p = std::sqrt(2.0 + d * d - 2.0 * std::cos(a - b) + 2.0 * d * (sa - sb));
    const double q = dubins::Helper::mod2pi(b - aux);

    const double length = (t + p + q) * m_radius;

    return {t, p, q, length, {Direction::LEFT, Direction::STRAIGHT, Direction::LEFT}};
}

Maneuver DubinsCurve::rsr(const double a, const double b, const double d, const double sa, const double ca, const double sb, const double cb) const noexcept
{
    const double aux = std::atan2(ca - cb, d - sa + sb);

    const double t = dubins::Helper::mod2pi(a - aux);
    const double p = std::sqrt(2.0 + d * d - 2.0 * std::cos(a - b) + 2.0 * d * (sb - sa));
    const double q = dubins::Helper::mod2pi(-dubins::Helper::mod2pi(b) + aux);

    const double length = (t + p + q) * m_radius;

    return {t, p, q, length, {Direction::RIGHT, Direction::STRAIGHT, Direction::RIGHT}};
}

Maneuver DubinsCurve::lsr(const double a, const double b, const double d, const double sa, const double ca, const double sb, const double cb) const noexcept
{
    const double aux1 = -2.0 + d * d + 2.0 * std::cos(a - b) + 2.0 * d * (sa + sb);

    if (aux1 < 0.0)
    {
        return {INFINITY, INFINITY, INFINITY, INFINITY, {Direction::NONE, Direction::NONE, Direction::NONE}};
    }

    const double p = std::sqrt(aux1);

    const double aux2 = std::atan2(-ca - cb, d + sa + sb) - std::atan(-2.0 / p);

    const double t = dubins::Helper::mod2pi(-a + aux2);
    const double q = dubins::Helper::mod2pi(-dubins::Helper::mod2pi(b) + aux2);

    const double length = (t + p + q) * m_radius;

    return {t, p, q, length, {Direction::LEFT, Direction::STRAIGHT, Direction::RIGHT}};
}

Maneuver DubinsCurve::rsl(const double a, const double b, const double d, const double sa, const double ca, const double sb, const double cb) const noexcept
{
    const double aux1 = d * d - 2.0 + 2.0 * std::cos(a - b) - 2.0 * d * (sa + sb);

    if (aux1 < 0.0)
    {
        return {INFINITY, INFINITY, INFINITY, INFINITY, {Direction::NONE, Direction::NONE, Direction::NONE}};
    }

    const double p = std::sqrt(aux1);

    const double aux2 = std::atan2(ca + cb, d - sa - sb) - std::atan(2.0 / p);

    const double t = dubins::Helper::mod2pi(a - aux2);
    const double q = dubins::Helper::mod2pi(dubins::Helper::mod2pi(b) - aux2);

    const double length = (t + p + q) * m_radius;

    return {t, p, q, length, {Direction::RIGHT, Direction::STRAIGHT, Direction::LEFT}};
}

Maneuver DubinsCurve::rlr(const double a, const double b, const double d, const double sa, const double ca, const double sb, const double cb) const noexcept
{
    const double aux = (6.0 - d * d + 2.0 * std::cos(a - b) + 2.0 * d * (sa - sb)) / 8.0;

    if (std::abs(aux) > 1.0)
    {
        return {INFINITY, INFINITY, INFINITY, INFINITY, {Direction::NONE, Direction::NONE, Direction::NONE}};
    }

    const double p = dubins::Helper::mod2pi(-std::acos(aux));
    const double t = dubins::Helper::mod2pi(a - std::atan2(ca - cb, d - sa + sb) + p / 2.0);
    const double q = dubins::Helper::mod2pi(a - b - t + p);

    const double length = (t + p + q) * m_radius;

    return {t, p, q, length, {Direction::RIGHT, Direction::LEFT, Direction::RIGHT}};
}

Maneuver DubinsCurve::lrl(const double a, const double b, const double d, const double sa, const double ca, const double sb, const double cb) const noexcept
{
    const double aux = (6.0 - d * d + 2.0 * std::cos(a - b) + 2.0 * d * (-sa + sb)) / 8.0;

    if (std::abs(aux) > 1.0)
    {
        return {INFINITY, INFINITY, INFINITY, INFINITY, {Direction::NONE, Direction::NONE, Direction::NONE}};
    }

    const double p = dubins::Helper::mod2pi(-std::acos(aux));
    const double t = dubins::Helper::mod2pi(-a + std::atan2(-ca + cb, d + sa - sb) + p / 2.0);
    const double q = dubins::Helper::mod2pi(dubins::Helper::mod2pi(b) - a - t + p);

    const double length = (t + p + q) * m_radius;

    return {t, p, q, length, {Direction::LEFT, Direction::RIGHT, Direction::LEFT}};
}

State DubinsCurve::getCoordinatesAt(const double offset) const
{
    const double noffset = offset / m_radius;
    State qir(0.0, 0.0, m_start.theta());
    State q;
    State q1;
    State q2;

    const double l1 = m_maneuver.m_t;
    const double l2 = m_maneuver.m_p;

    if (noffset < l1)
    {
        q = getPositionInSegment(noffset, qir, m_maneuver.m_combination[0]);
    }
    else if (noffset < (l1 + l2))
    {
        q1 = getPositionInSegment(l1, qir, m_maneuver.m_combination[0]);
        q = getPositionInSegment(noffset - l1, q1, m_maneuver.m_combination[1]);
    }
    else
    {
        q1 = getPositionInSegment(l1, qir, m_maneuver.m_combination[0]);
        q2 = getPositionInSegment(l2, q1, m_maneuver.m_combination[1]);
        q = getPositionInSegment(noffset - l1 - l2, q2, m_maneuver.m_combination[2]);
    }

    const double poseX = q.point().x() * m_radius + m_start.point().x();
    const double poseY = q.point().y() * m_radius + m_start.point().y();
    const double theta = dubins::Helper::mod2pi(q.theta());

    return {poseX, poseY, theta};
}

State DubinsCurve::getPositionInSegment(const double offset, const State& start, const Direction direction) const noexcept
{
    const double startPoseX = start.point().x();
    const double startPoseY = start.point().y();
    const double startTheta = start.theta();

    double poseX = startPoseX;
    double poseY = startPoseY;
    double theta = startTheta;

    switch (direction)
    {
    case Direction::LEFT:
        poseX = startPoseX + std::sin(startTheta + offset) - std::sin(startTheta);
        poseY = startPoseY - std::cos(startTheta + offset) + std::cos(startTheta);
        theta = startTheta + offset;
        break;
    case Direction::RIGHT:
        poseX = startPoseX - std::sin(startTheta - offset) + std::sin(startTheta);
        poseY = startPoseY + std::cos(startTheta - offset) - std::cos(startTheta);
        theta = startTheta - offset;
        break;
    case Direction::STRAIGHT:
        poseX = startPoseX + std::cos(startTheta) * offset;
        poseY = startPoseY + std::sin(startTheta) * offset;
        theta = startTheta;
        break;
    default:
        break;
    }

    return {poseX, poseY, theta};
}

} // namespace dubins
