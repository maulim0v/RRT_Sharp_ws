#ifndef NODE_COMPARE_HPP
#define NODE_COMPARE_HPP

// Component
#include "KeyCompare.hpp"

// Standard
#include <memory>

class Node;

/**
 *
 */
class NodeCompare
{
public:
    /**
     * Key comparison functor
     * @param nodeLeft Left node (f/g) to compare with
     * @param nodeRight Right node (f/g) to compare with
     *
     * @return True if left node > right node
     */
    bool operator()(Node* nodeLeft, Node* nodeRight) const noexcept;

private:
    KeyCompare m_compare; ///< Key comapre
};

#endif // NODE_COMPARE_HPP
