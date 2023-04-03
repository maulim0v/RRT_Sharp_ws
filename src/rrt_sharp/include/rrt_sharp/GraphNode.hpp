#ifndef GRAPH_NODE_HPP
#define GRAPH_NODE_HPP

// Component
#include "Point.hpp"
#include "State.hpp"

/**
 * @brief The GraphNode class
 */
class GraphNode
{
public:
    /**
     * @brief GraphNode
     */
    GraphNode();

    /**
     * @brief GraphNode
     * @param p
     * @param theta
     * @param rho
     * @param id
     * @param parentId
     */
    GraphNode(const Point &p, const double theta, const double rho, const int id, const int parentId);

    /**
     * @brief GraphNode
     * @param x
     * @param y
     * @param z
     * @param theta
     * @param rho
     * @param id
     * @param parentId
     */
    GraphNode(const double x, const double y, const double z, const double theta, const double rho, const int id, const int parentId);

    /**
     * @brief id
     * @return
     */
    int id() const noexcept;

    /**
     * @brief setId
     * @param id
     */
    void setId(int id) noexcept;

    /**
     * @brief parentId
     * @return
     */
    int parentId() const noexcept;

    /**
     * @brief setParentId
     * @param parentId
     */
    void setParentId(int parentId) noexcept;
protected:

    /**
     * @brief buildGraphNode
     */
    void buildGraphNode();

    /**
     * @brief buildGraphNode
     * @param n
     */
    void buildGraphNode(const GraphNode& n);

    /**
     * @brief buildGraphNode
     * @param p
     * @param theta
     * @param rho
     * @param id
     * @param parentId
     */
    void buildGraphNode(const Point& p, const double theta, const double rho, const int id, const int parentId);

    /**
     * @brief buildGraphNode
     * @param x
     * @param y
     * @param z
     * @param theta
     * @param rho
     * @param id
     * @param parentId
     */
    void buildGraphNode(const double x, const double y, const double z, const double theta, const double rho, const int id, const int parentId);

    int m_id; ///<
    int m_parentId; ///<
};

#endif // GRAPH_NODE_HPP
