// Component
#include "NodeCompare.hpp"

#include "Node.hpp"

bool NodeCompare::operator()(Node* nodeLeft, Node* nodeRight) const noexcept
{
    return (m_compare(nodeLeft->key(), nodeRight->key()) == false);
}
