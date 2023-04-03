#ifndef EDGE_HPP
#define EDGE_HPP

// Component
#include "GraphNode.hpp"

/**
 * @brief The Edge class
 */
class Edge
{
public:
    /**
     * @brief Edge
     */
    Edge();

    /**
     * @brief Edge
     * @param start
     * @param end
     */
    Edge(const GraphNode& start, const GraphNode& end);

    /**
     * @brief start
     * @return
     */
    const GraphNode& start() const noexcept;

    /**
     * @brief end
     * @return
     */
    const GraphNode& end() const noexcept;
protected:
    GraphNode m_start; ///<
    GraphNode m_end; ///<
};

#endif // EDGE_HPP
