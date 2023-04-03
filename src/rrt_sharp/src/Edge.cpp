// Component
#include "Edge.hpp"

#include "GraphNode.hpp"

Edge::Edge()
{}

Edge::Edge(const GraphNode& start, const GraphNode& end) :
    m_start{start},
    m_end{end}
{}

const GraphNode& Edge::start() const noexcept
{
    return m_start;
}

const GraphNode& Edge::end() const noexcept
{
    return m_end;
}
