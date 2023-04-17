#ifndef RRT_SHARP_HPP
#define RRT_SHARP_HPP

// Component
#include "Edge.hpp"
#include "KeyCompare.hpp"
#include "Node.hpp"
#include "NodeCompare.hpp"
#include "State.hpp"

// Library
#include <boost/heap/fibonacci_heap.hpp>
#include <eigen3/Eigen/Core>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>

// Standard
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

class RRTSharp final
{
public:

    RRTSharp(const std::vector<double>& objectMap = {});

    void setMapInfo(const double minX, const double minY, const double resolution, const int numX, const int numY);
    void setObjectMap(const std::vector<double>& objectMap);
    void setObservedMap(const std::vector<int>& observedMap);
    void setTerrainMap(const std::vector<double>& terrainMap);
    void setStartState(const double x, const double y, const double thetaDeg);
    void setGoalState(const double x, const double y, const double thetaDeg);
    void setStopRadius(const double radius);
    void init();
    void run();

    std::vector<double> getTrajectoryX() const noexcept;
    std::vector<double> getTrajectoryY() const noexcept;
    std::vector<double> getSearchSpaceX() const noexcept;
    std::vector<double> getSearchSpaceY() const noexcept;

private:

    void initSearchSpace() noexcept;
    void initObjects(const bool run = false) noexcept;
    void initVariables(const bool run = false) noexcept;
    void initGraph() noexcept;
    void addObject(const double minX, const double maxX, const double minY, const double maxY) noexcept;
    void showSearchSpace() noexcept;
    void plotSearchSpace() noexcept;
    double calculateRadius() const noexcept;
    void plotStartState() noexcept;
    void plotState(const State& state) noexcept;
    void plotPath(const std::vector<State>& path) noexcept;
    void plotPathFromIndeces(const std::vector<std::pair<int, int>>& pathIndeces) noexcept;
    void plotGoalState() noexcept;
    void debugDubinsPath() noexcept;
    void addNode(const std::shared_ptr<Node>& node) noexcept;
    double calculateCost(const std::vector<std::pair<int, int>>& pathIndeces, const double pathLength);
    void plotTree() const noexcept;
    void plotEdge(const Point& start, const Point& end) const noexcept;

    State sampleState() const noexcept;

    const Node& nearest(const std::shared_ptr<Node>& node) const noexcept;
    std::shared_ptr<Node> sampleNode() const noexcept;
    void interpolateNode(Node& node, const Node& compare) const noexcept;
    bool checkNode(const Node& node) const noexcept;
    bool checkPathIndeces(const std::vector<std::pair<int, int>>& pathIndeces) const noexcept;
    bool checkPath(const std::vector<State>& path) const noexcept;
    void near(const std::shared_ptr<Node>& node, std::vector<std::shared_ptr<Node>>& nearNodes) const noexcept;
    Key key(const Node& node) const noexcept;
    double heuristic(const Node& node) const noexcept;
    void updateQueue(const std::shared_ptr<Node>& node);

    double m_minX; ///<
    double m_maxX; ///<
    double m_minY; ///<
    double m_maxY; ///<
    double m_resolution; ///<
    int m_numX; ///<
    int m_numY; ///<
    int m_numGrids; ///<

    std::vector<double> m_objectMap;
    std::vector<int> m_observedMap;
    std::vector<double> m_terrainMap;

    int m_numOccupiedGrids; ///<

    std::vector<bool> m_searchMap; ///< False is free grid, True is occupied grid

    double m_gammaStar; ///<

    int m_numExpandedGrids; ///<

    State m_startState; ///<
    State m_goalState; ///<
    double m_goalRaduis; ///<

    std::unordered_map<int, std::shared_ptr<Node>> m_nodeById;
    std::vector<Edge> m_edges;

    std::vector<double> m_optimalTrajectoryX;
    std::vector<double> m_optimalTrajectoryY;

    std::unordered_map<int, std::vector<int>> m_childrenByParent;

    std::vector<bool> m_expandedGridMap;

    boost::heap::fibonacci_heap<Node*, boost::heap::compare<NodeCompare>> m_queue;
    KeyCompare m_keyCompare;
    Cost m_bestCost;
    Node m_bestNode;

    std::unordered_set<void*> m_erasedPointers;
    std::unordered_set<void*> m_addPointers;

    std::shared_ptr<ompl::NearestNeighborsGNATNoThreadSafety<std::shared_ptr<Node>>> m_nn; ///<
};

#endif // RRT_SHARP_HPP
