// Component
#include "GraphNode.hpp"

GraphNode::GraphNode()
{
    buildGraphNode();
}

GraphNode::GraphNode(const Point& p, const double theta, const double rho, const int id, const int parentId)
{
    buildGraphNode(p, theta, rho, id, parentId);
}

GraphNode::GraphNode(const double x, const double y, const double z, const double theta, const double rho, const int id, const int parentId)
{
    buildGraphNode(x, y, z, theta, rho, id, parentId);
}

void GraphNode::buildGraphNode()
{
//    m_x = 0.0;
//    m_y = 0.0;
//    m_z = 0.0;
//    m_theta = 0.0;
//    m_rho = 0.0;
    m_id = 0;
    m_parentId = 0;
}

void GraphNode::buildGraphNode(const GraphNode& n)
{
//    m_x = n.m_x;
//    m_y = n.m_y;
//    m_z = n.m_z;
//    m_theta = n.m_theta;
//    m_rho = n.m_rho;
    m_id = n.m_id;
    m_parentId = n.m_parentId;
}

void GraphNode::buildGraphNode(const Point& p, const double theta, const double rho, const int id, const int parentId)
{
//    m_x = p.x();
//    m_y = p.y();
//    m_z = p.z();
//    m_theta = theta;
//    m_rho = rho;
    m_id = id;
    m_parentId = parentId;
}

void GraphNode::buildGraphNode(const double x, const double y, const double z, const double theta, const double rho, const int id, const int parentId)
{
//    m_x = x;
//    m_y = y;
//    m_z = z;
//    m_theta = theta;
//    m_rho = rho;
    m_id = id;
    m_parentId = parentId;
}

int GraphNode::id() const noexcept
{
    return m_id;
}

int GraphNode::parentId() const noexcept
{
    return m_parentId;
}

void GraphNode::setId(int id) noexcept
{
    m_id = id;
}

void GraphNode::setParentId(int parentId) noexcept
{
    m_parentId = parentId;
}
