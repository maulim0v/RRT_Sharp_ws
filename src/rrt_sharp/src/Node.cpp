// Component
#include "Node.hpp"

Node::Node() :
    m_key{0.0, 0.0}
{
    buildGraphNode();
}

Node::Node(const Point& p, const double theta, const double rho, const int id, const int parentId) :
    m_key{0.0, 0.0}
{
    buildGraphNode();
}

Node::Node(const double x, const double y, const double z, const double theta, const double rho, const int id, const int parentId) :
    m_key{0.0, 0.0}
{
    buildGraphNode();
}

Node::Node(const State& state, const int id, const int parentId, const Cost& g, const Cost& lmc) :
    m_state{state},
    m_id{id},
    m_parentId{parentId},
    m_g{g},
    m_lmc{lmc},
    m_key{0.0, 0.0}
{}

void Node::buildGraphNode()
{
//    m_x = 0.0;
//    m_y = 0.0;
//    m_z = 0.0;
//    m_theta = 0.0;
//    m_rho = 0.0;
    m_id = 0;
    m_parentId = 0;
}

int Node::id() const noexcept
{
    return m_id;
}

int Node::parentId() const noexcept
{
    return m_parentId;
}

void Node::setId(int id) noexcept
{
    m_id = id;
}

void Node::setParentId(int parentId) noexcept
{
    m_parentId = parentId;
}

const State& Node::state() const noexcept
{
    return m_state;
}

void Node::setX(const double x) noexcept
{
    m_state.setX(x);
}

void Node::setY(const double y) noexcept
{
    m_state.setY(y);
}

const Cost& Node::g() const noexcept
{
    return m_g;
}

void Node::setG(const Cost& cost) noexcept
{
    m_g = cost;
}

const Cost& Node::lmc() const noexcept
{
    return m_lmc;
}

void Node::setLmc(const Cost& lmc) noexcept
{
    m_lmc = lmc;
}

const Key& Node::key() const noexcept
{
    return m_key;
}

void Node::setKey(const Key& key) noexcept
{
    m_key = key;
}
