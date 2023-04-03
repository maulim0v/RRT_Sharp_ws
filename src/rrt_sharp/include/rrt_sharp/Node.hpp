#ifndef NODE_HPP
#define NODE_HPP

// Component
#include "Cost.hpp"
#include "Key.hpp"
#include "NodeCompare.hpp"
#include "Point.hpp"
#include "State.hpp"

// Library
#include <boost/heap/fibonacci_heap.hpp>

// Standard
#include <cmath>
#include <memory>

/**
 *
 */
class Node final
{
public:
    /**
     * @brief GraphNode
     */
    Node();

    /**
     * @brief GraphNode
     * @param p
     * @param theta
     * @param rho
     * @param id
     * @param parentId
     */
    Node(const Point &p, const double theta, const double rho, const int id, const int parentId);

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
    Node(const double x, const double y, const double z, const double theta, const double rho, const int id, const int parentId);

    /**
     * @brief Node
     * @param state
     * @param id
     * @param parentId
     * @param g
     * @param lmc
     */
    Node(const State& state, const int id, const int parentId = -1, const Cost& g = Cost(INFINITY), const Cost& lmc = Cost(INFINITY));

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

    /**
     * @brief state
     * @return
     */
    const State& state() const noexcept;

    /**
     * @brief setX
     * @param x
     */
    void setX(const double x) noexcept;

    /**
     * @brief setY
     * @param y
     */
    void setY(const double y) noexcept;

    /**
     * @brief g
     * @return
     */
    const Cost& g() const noexcept;

    /**
     * @brief setG
     * @param cost
     */
    void setG(const Cost& cost) noexcept;

    /**
     * @brief lmc
     * @return
     */
    const Cost& lmc() const noexcept;

    /**
     * @brief setLmc
     * @param lmc
     */
    void setLmc(const Cost& lmc) noexcept;

    /**
     * @brief key
     * @return
     */
    const Key& key() const noexcept;

    /**
     * @brief setKey
     * @param key
     */
    void setKey(const Key& key) noexcept;

    boost::heap::fibonacci_heap<Node*, boost::heap::compare<NodeCompare>>::handle_type m_handle;

private:
    /**
     * @brief buildGraphNode
     */
    void buildGraphNode();

    State m_state; ///<
    int m_id; ///<
    int m_parentId; ///<
    Cost m_g;
    Cost m_lmc;
    Key m_key;

};

#endif // NODE_HPP
