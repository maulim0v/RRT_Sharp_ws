// Component
#include "RRTSharp.hpp"

#include "dubins/DubinsCurve.hpp"
#include "Ray.hpp"

// Library
#include <matplotlibcpp.h>

// Standard
#include <algorithm>
#include <cmath>
#include <chrono>
#include <random>
#include <unordered_set>

RRTSharp::RRTSharp(const std::vector<double>& objectMap) :
    m_minX{-100.0},
    m_maxX{100.0},
    m_minY{-100.0},
    m_maxY{100.0},
    m_resolution{1.0},
    m_numX{200},
    m_numY{200},
    m_numGrids{m_numX * m_numY},
    m_numOccupiedGrids{0},
    m_gammaStar{0.0},
    m_numExpandedGrids{1},
    m_bestCost{INFINITY},
    m_bestNode{},
    m_nn(std::make_shared<ompl::NearestNeighborsGNATNoThreadSafety<std::shared_ptr<Node>>>())
{
    // Set distance function
    m_nn->setDistanceFunction([](const std::shared_ptr<Node>& left, const std::shared_ptr<Node>& right){ return left->state().point().distanceTo(right->state().point()); });

    return;

    // Initialize search space, 2D free/occupied map
    initSearchSpace();

    // Set map
    m_expandedGridMap.resize(m_numGrids, false);

    // Initialize test objects
    initObjects();

    // Initialize variables (gamma star)
    initVariables();

    // Im Show search space
//    showSearchSpace();

    // Plot search space
    plotSearchSpace();

    // Plot start state
    plotStartState();

    // Plot goal state
    plotGoalState();

    // Debug dubins path
//    debugDubinsPath();


    // Initialize the graph
    initGraph();

    auto t100 = std::chrono::system_clock::now();

    constexpr int maxIterations = 20000;
    for (int iteration = 0; iteration < maxIterations; ++iteration)
    {
        // Sample node
        auto sample = sampleNode();

        // Closest node
        const Node& closest = nearest(sample);

        // Interpolate node
        interpolateNode(*sample, closest);

        // Generate ray from closest to sample state
        const Ray ray(closest.state().point(), sample->state().point(), m_resolution, m_minX, m_minY);

        // Generate path based on dubins maneuver
        const std::vector<std::pair<int, int>>& pathIndeces = ray.indeces();

//        std::cout << "pathSize: " << path.size() << std::endl;
//        plotPathFromIndeces(pathIndeces);
//        matplotlibcpp::xlim(m_minX, m_maxX);
//        matplotlibcpp::ylim(m_minY, m_maxY);
//        matplotlibcpp::show();

        // Check path, is obstacle free
        if (checkPathIndeces(pathIndeces) == false)
        {
            continue; // Try another sample
        }

        // Plot state for debugging purposes
//        plotState(sample->state());

        // Nodes closeby
        std::vector<std::shared_ptr<Node>> close;
        near(sample, close);

        // No need to continue if no close neighbors found
        if (close.empty() == true)
        {
            std::cout << "nothing closeby try again" << std::endl;
            continue;
        }

        // Check for better cost
        for (const auto& neighbor : close)
        {
            const int id = neighbor->id();

            // Generate ray from closest to sample state
            const Ray ray(neighbor->state().point(), sample->state().point(), m_resolution, m_minX, m_minY);

            // Generate path based on dubins maneuver
            const std::vector<std::pair<int, int>>& pathIndeces = ray.indeces();

            // Check path, is obstacle free
            if (checkPathIndeces(pathIndeces) == false)
            {
                continue; // Try another neighbor
            }

            // Calculate path cost
            const double c = ray.length();

            // Calculate g cost
            const double g = c + neighbor->g().value();

            // Set g
            sample->setG(Cost(g));

            // Update lmc
            if (sample->lmc().value() > g)
            {
                sample->setLmc(Cost(g));
                sample->setParentId(id);
            }

            // Add edge
            m_childrenByParent[id].push_back(sample->id());
            m_childrenByParent[sample->id()].push_back(id);
        }

        // Add node
        addNode(sample);

        // Update queue
        updateQueue(sample);

        if (sample->lmc().value() < m_bestCost.value())
        {
            m_bestCost = sample->lmc();
            m_bestNode = *sample;
        }

        // Replan
        while (true)
        {
            if (m_queue.empty() == true)
            {
                m_erasedPointers.clear();
                m_addPointers.clear();
                break;
            }

            Node* min = m_queue.top();

//            if (m_keyCompare(min->key(), m_bestNode.key()) == false)
//            {
                //break;
//            }

            min->setG(min->lmc());

            m_queue.erase(min->m_handle);
            m_erasedPointers.insert(min->m_handle.node_);

            const int childrenSize = m_childrenByParent.at(min->id()).size();
            for (int succ = 0; succ < childrenSize; ++succ)
            {
                const int childId = m_childrenByParent.at(min->id()).at(succ);
                const auto& childNode = m_nodeById.at(childId);

                const Ray ray(childNode->state().point(), min->state().point(), m_resolution, m_minX, m_minY);
                const double c = ray.length();
                const double g = c + min->g().value();

                if (g < childNode->lmc().value())
                {
                    childNode->setLmc(Cost(g));
                    childNode->setParentId(min->id());
                    updateQueue(childNode);
                }
            }
        }

        const double distanceToStart = sample->state().point().distanceTo(m_startState.point());
        if (distanceToStart < 2.0)
        {
            std::cout << "Reached" << std::endl;
            break;
        }
    }

    std::cout << "numNodes: " << m_nodeById.size()
              << ", numChildren: " << m_childrenByParent.size()
              << std::endl;
    auto t101 = std::chrono::system_clock::now();
    float elapsedTotal = static_cast<float>(std::chrono::duration_cast <std::chrono::microseconds>(t101 - t100).count()) / 1000.0f;
    std::cout << "total: " << elapsedTotal << " ms"
              << std::endl;

    if (true)
    {
        std::unordered_set<int> exists;
        for (const auto& nodeId : m_nodeById)
        {
            int selfId = nodeId.first;
            int parentId = nodeId.second->parentId();

            if (exists.count(selfId) > 0)
            {
                continue;
            }

            while (parentId != -1)
            {
                if (exists.count(selfId) > 0)
                {
                    break;
                }

                if (true &&
                    (std::abs(m_nodeById.at(selfId)->g().value() - m_nodeById.at(selfId)->lmc().value()) < 0.01) &&
                    (std::abs(m_nodeById.at(parentId)->g().value() - m_nodeById.at(parentId)->lmc().value()) < 0.01))
                {
                    plotEdge(m_nodeById.at(selfId)->state().point(), m_nodeById.at(parentId)->state().point());
                }

                exists.emplace(selfId);

                selfId = m_nodeById.at(parentId)->id();
                parentId = m_nodeById.at(parentId)->parentId();
            }
        }

        matplotlibcpp::xlim(m_minX, m_maxX);
        matplotlibcpp::ylim(m_minY, m_maxY);
    }

    matplotlibcpp::show();
}

void RRTSharp::setMapInfo(const double minX, const double minY, const double resolution, const int numX, const int numY)
{
    m_minX = minX;
    m_minY = minY;

    m_numX = numX;
    m_numY = numY;
    m_numGrids = numX * numY;

    m_resolution = resolution;

    m_maxX = minX + m_resolution * static_cast<double>(numX);
    m_maxY = minY + m_resolution * static_cast<double>(numY);
}

void RRTSharp::setObjectMap(const std::vector<double>& objectMap)
{
    m_objectMap = objectMap;
}

void RRTSharp::setObservedMap(const std::vector<int>& observedMap)
{
    m_observedMap = observedMap;
}

void RRTSharp::setTerrainMap(const std::vector<double>& terrainMap)
{
    m_terrainMap = terrainMap;
}

void RRTSharp::setStartState(const double x, const double y, const double thetaDeg)
{
    // Initialize start state
    m_startState = State(x, y, thetaDeg * M_PI / 180.0);
}

void RRTSharp::setGoalState(const double x, const double y, const double thetaDeg)
{
    // Initialize goal state
    m_goalState = State(x, y, thetaDeg * M_PI / 180.0);
}

void RRTSharp::setStopRadius(const double radius)
{
    m_goalRaduis = radius;
}

void RRTSharp::init()
{
    // Initialize search space, 2D free/occupied map
    initSearchSpace();

    // Set map
    m_expandedGridMap.resize(m_numGrids, false);

    // Initialize test objects
    initObjects(true);

    // Initialize variables (gamma star)
    initVariables(true);

    // Plot search space
    plotSearchSpace();

    // Plot start state
    plotStartState();

    // Plot goal state
    plotGoalState();

    // Initialize the graph
    initGraph();
}

void RRTSharp::run()
{
    auto t100 = std::chrono::system_clock::now();
    constexpr int maxIterations = 50000;
    for (int iteration = 0; iteration < maxIterations; ++iteration)
    {
//        std::cout << "iter: " << iteration << std::endl;

        // Sample node
        auto sample = sampleNode();

//        std::cout << "a1" << std::endl;

        // Closest node
        const Node& closest = nearest(sample);

//        std::cout << "a2" << std::endl;

        // Interpolate node
        interpolateNode(*sample, closest);

//        std::cout << "a3" << std::endl;

        // Generate ray from closest to sample state
        const Ray ray(closest.state().point(), sample->state().point(), m_resolution, m_minX, m_minY);

//        std::cout << "a4" << std::endl;

        // Generate path based on dubins maneuver
        const std::vector<std::pair<int, int>>& pathIndeces = ray.indeces();

//        std::cout << "a5" << std::endl;

//        std::cout << "pathSize: " << path.size() << std::endl;
//        plotPathFromIndeces(pathIndeces);
//        matplotlibcpp::xlim(m_minX, m_maxX);
//        matplotlibcpp::ylim(m_minY, m_maxY);
//        matplotlibcpp::show();

        // Check path, is obstacle free
        if (checkPathIndeces(pathIndeces) == false)
        {
            continue; // Try another sample
        }

//        std::cout << "a6" << std::endl;

        // Plot state for debugging purposes
//        plotState(sample->state());

        // Nodes closeby
        std::vector<std::shared_ptr<Node>> close;
        near(sample, close);

//        std::cout << "a7" << std::endl;

        // No need to continue if no close neighbors found
        if (close.empty() == true)
        {
            std::cout << "nothing closeby try again" << std::endl;
            continue;
        }

        // Check for better cost
        for (const auto& neighbor : close)
        {
            const int id = neighbor->id();

            // Generate ray from closest to sample state
            const Ray ray(neighbor->state().point(), sample->state().point(), m_resolution, m_minX, m_minY);

            // Generate path based on dubins maneuver
            const std::vector<std::pair<int, int>>& pathIndeces = ray.indeces();

            // Check path, is obstacle free
            if (checkPathIndeces(pathIndeces) == false)
            {
                continue; // Try another neighbor
            }

            // Calculate path cost
            const double c = calculateCost(pathIndeces, ray.length());

            // Calculate g cost
            const double g = c + neighbor->g().value();

            // Set g
            sample->setG(Cost(g));

            // Update lmc
            if (sample->lmc().value() > g)
            {
                sample->setLmc(Cost(g));
                sample->setParentId(id);
            }

            // Add edge
            m_childrenByParent[id].push_back(sample->id());
            m_childrenByParent[sample->id()].push_back(id);
        }

//        std::cout << "a8" << std::endl;

        // Add node
        addNode(sample);

//        std::cout << "a9" << std::endl;

        // Update queue
        updateQueue(sample);

//        std::cout << "a10" << std::endl;

        if (sample->lmc().value() < m_bestCost.value())
        {
            m_bestCost = sample->lmc();
            m_bestNode = *sample;
        }

        // Replan
        while (true)
        {
            if (m_queue.empty() == true)
            {
                m_erasedPointers.clear();
                m_addPointers.clear();
                break;
            }

            Node* min = m_queue.top();

//            if (m_keyCompare(min->key(), m_bestNode.key()) == false)
//            {
                //break;
//            }

            min->setG(min->lmc());

            m_queue.erase(min->m_handle);
            m_erasedPointers.insert(min->m_handle.node_);

            const int childrenSize = m_childrenByParent.at(min->id()).size();
            for (int succ = 0; succ < childrenSize; ++succ)
            {
                const int childId = m_childrenByParent.at(min->id()).at(succ);
                const auto& childNode = m_nodeById.at(childId);

                const Ray ray(childNode->state().point(), min->state().point(), m_resolution, m_minX, m_minY);
                const double c = calculateCost(pathIndeces, ray.length());
                const double g = c + min->g().value();

                if (g < childNode->lmc().value())
                {
                    childNode->setLmc(Cost(g));
                    childNode->setParentId(min->id());
                    updateQueue(childNode);
                }
            }
        }

//        std::cout << "a11" << std::endl;

        const double distanceToStart = sample->state().point().distanceTo(m_startState.point());
        if (distanceToStart < 2.0)
        {
            std::cout << "Reached" << std::endl;
            //break;
        }
    }

    std::cout << "numNodes: " << m_nodeById.size()
              << ", numChildren: " << m_childrenByParent.size()
              << std::endl;
    auto t101 = std::chrono::system_clock::now();
    float elapsedTotal = static_cast<float>(std::chrono::duration_cast <std::chrono::microseconds>(t101 - t100).count()) / 1000.0f;
    std::cout << "total: " << elapsedTotal << " ms"
              << std::endl;

    if (true)
    {
        std::unordered_set<int> exists;
        for (const auto& nodeId : m_nodeById)
        {
            int selfId = nodeId.first;
            int parentId = nodeId.second->parentId();

            if (exists.count(selfId) > 0)
            {
                continue;
            }

            while (parentId != -1)
            {
                if (exists.count(selfId) > 0)
                {
                    break;
                }

                if (true &&
                    (std::abs(m_nodeById.at(selfId)->g().value() - m_nodeById.at(selfId)->lmc().value()) < 0.01) &&
                    (std::abs(m_nodeById.at(parentId)->g().value() - m_nodeById.at(parentId)->lmc().value()) < 0.01))
                {
                    plotEdge(m_nodeById.at(selfId)->state().point(), m_nodeById.at(parentId)->state().point());
                }

                exists.emplace(selfId);

                selfId = m_nodeById.at(parentId)->id();
                parentId = m_nodeById.at(parentId)->parentId();
            }
        }
    }

    matplotlibcpp::xlim(m_minX, m_maxX);
    matplotlibcpp::ylim(m_minY, m_maxY);
    matplotlibcpp::show();
}

double RRTSharp::calculateCost(const std::vector<std::pair<int, int>>& pathIndeces, const double pathLength)
{
    double cost = pathLength;

    for (const auto& xyIndex : pathIndeces)
    {
        const int index = xyIndex.first * m_numY + xyIndex.second;

        double cellCost = 10.0 * m_objectMap.at(index) * m_objectMap.at(index);
        if (m_observedMap.at(index) == 4)
        {
            cellCost += 50.0;
        }

        cost += cellCost;
    }

    return cost;
}

void RRTSharp::plotTree() const noexcept
{
    std::map<std::string, std::string> keywords;
    keywords.emplace("color", "black");
    keywords.emplace("marker", "s");

    std::unordered_map<int, std::unordered_set<int>> exists;
    for (const auto& children : m_childrenByParent)
    {
        for (const auto& child : children.second)
        {
            if (exists.find(children.first) != std::cend(exists))
            {
                if (exists.at(children.first).count(child) > 0)
                {
                    continue;
                }
            }

            if (exists.find(child) != std::cend(exists))
            {
                if (exists.at(child).count(children.first) > 0)
                {
                    continue;
                }
            }

            std::vector<double> pathX{m_nodeById.at(children.first)->state().point().x(), m_nodeById.at(child)->state().point().x()};
            std::vector<double> pathY{m_nodeById.at(children.first)->state().point().y(), m_nodeById.at(child)->state().point().y()};

            exists[children.first].insert(child);
            exists[child].insert(children.first);

            matplotlibcpp::plot(pathX, pathY);
        }
    }

//    matplotlibcpp::show();
}

void RRTSharp::plotEdge(const Point& start, const Point& end) const noexcept
{
    std::map<std::string, std::string> keywords;
    keywords.emplace("color", "cyan");
//    keywords.emplace("marker", "s");

    std::vector<double> pathX{start.x(), end.x()};
    std::vector<double> pathY{start.y(), end.y()};

    matplotlibcpp::plot(pathX, pathY);
}

void RRTSharp::addNode(const std::shared_ptr<Node>& node) noexcept
{
    m_nodeById[node->id()] = node;
    m_nn->add(node);

    const int indexX = static_cast<int>(std::floor((node->state().point().x() - m_minX) / m_resolution));
    const int indexY = static_cast<int>(std::floor((node->state().point().y() - m_minY) / m_resolution));
    const int index = indexX * m_numY + indexY;

//    std::cout << "x: " << node->state().point().x()
//              << ", y: " << node->state().point().y()
//              << ", indX: " << indexX
//              << ", indY: " << indexY
//              << std::endl;

    if (m_expandedGridMap.at(index) == false)
    {
        ++m_numExpandedGrids;
        m_expandedGridMap[index] = true;
    }
}

void RRTSharp::near(const std::shared_ptr<Node>& node, std::vector<std::shared_ptr<Node>>& nearNodes) const noexcept
{
//    std::vector<int> nodeIds;

    // Calculate radius to search for nearby nodes
    const double radius = 0.5 * calculateRadius();

//    std::cout << "radius neighbor: " << radius << std::endl;

    // Check for nodes within the search radius
//    for (const auto& nodeId : m_nodeById)
//    {
//        const double distance = nodeId.second->state().point().distanceTo(node.state().point());

//        if (distance < radius)
//        {
//            nodeIds.push_back(nodeId.first);
//        }
//    }

//    std::cout << "numNeighbors: " << nodeIds.size() << std::endl;

    m_nn->nearestK(node, 20, nearNodes);
//    return nodeIds;
}

bool RRTSharp::checkNode(const Node& node) const noexcept
{
    const int indexX = static_cast<int>(std::floor(node.state().point().x() / m_resolution));
    const int indexY = static_cast<int>(std::floor(node.state().point().y() / m_resolution));

    // Outside of the map, should not really happen
    if ((indexX < 0) ||
        (indexX >= m_numX) ||
        (indexY < 0) ||
        (indexX >= m_numY))
    {
        return false;
    }

    const int index = indexX * m_numY + indexY;

    // Occupied, not free space
    if (m_searchMap.at(index) == true)
    {
        return false;
    }

    return true;
}

bool RRTSharp::checkPathIndeces(const std::vector<std::pair<int, int>>& pathIndeces) const noexcept
{
    if (pathIndeces.empty() == true)
    {
        std::cout << "Path empty" << std::endl;
        return false;
    }

    for (const auto& xy : pathIndeces)
    {
        const int index = xy.first * m_numY + xy.second;

        // Outside boundary
        if ((xy.first < 0) ||
            (xy.first >= m_numX) ||
            (xy.second < 0) ||
            (xy.second >= m_numY))
        {
            return false;
        }

        // Occupied, not free space
        if (m_searchMap.at(index) == true)
        {
//            std::cout << "Path occudied" << std::endl;
            return false;
        }
    }

    return true;
}

bool RRTSharp::checkPath(const std::vector<State>& path) const noexcept
{
    if (path.empty() == true)
    {
        std::cout << "Path empty" << std::endl;
        return false;
    }

    for (const auto& state : path)
    {
        const int indexX = static_cast<int>(std::floor(state.point().x() / m_resolution));
        const int indexY = static_cast<int>(std::floor(state.point().y() / m_resolution));

        // Outside of the map
        if ((indexX < 0) ||
            (indexX >= m_numX) ||
            (indexY < 0) ||
            (indexY >= m_numY))
        {
            std::cout << "Path outside map" << std::endl;
            return false;
        }

        const int index = indexX * m_numY + indexY;

        // Occupied, not free space
        if (m_searchMap.at(index) == true)
        {
            std::cout << "Path occudied" << std::endl;
            return false;
        }
    }

    return true;
}

void RRTSharp::interpolateNode(Node& node, const Node& compare) const noexcept
{
    // Maximum norm2 expansion distance
    static constexpr double maxDistance = 5.0;

    // Distance to the closest state
    const double distance = compare.state().point().distanceTo(node.state().point());

    // Check distance
    if (distance < maxDistance)
    {
        return;
    }

    // Interpolate x and y
    const double ratio = maxDistance / distance;
    const Point diff = node.state().point() - compare.state().point();
    node.setX(compare.state().point().x() + ratio * diff.x());
    node.setY(compare.state().point().y() + ratio * diff.y());
}

const Node& RRTSharp::nearest(const std::shared_ptr<Node>& node) const noexcept
{
//    double nearestDistance = 10.0 * m_numX * m_resolution;
//    int nearestId = 0;

//    for (const auto& nodeId : m_nodeById)
//    {
//        const double distance = nodeId.second->state().point().distanceTo(node.point());

//        if (distance < nearestDistance)
//        {
//            nearestDistance = distance;
//            nearestId = nodeId.first;
//        }
//    }

    return *m_nn->nearest(node);
//    return *m_nodeById.at(nearestId);
}

std::shared_ptr<Node> RRTSharp::sampleNode() const noexcept
{
    static int NODE_ID = 1;

    const State state = sampleState();
    const Node node(state, NODE_ID);

    ++NODE_ID;

    return std::make_shared<Node>(node);
}

void RRTSharp::debugDubinsPath() noexcept
{
    State start(10.0, 10.0, 10.0 * M_PI / 180.0);// = sampleState();
    State end(20.0, 50.0, 330.0 * M_PI / 180.0);// = sampleState();

    dubins::DubinsCurve path(start, end, 10.0);
    const std::vector<State> states = path.getPath(0.1);

    std::vector<double> vecX;
    std::vector<double> vecY;

    for (const auto& point : states)
    {
        vecX.push_back(point.point().x());
        vecY.push_back(point.point().y());
    }

    matplotlibcpp::scatter(vecX, vecY);
    matplotlibcpp::axis("equal");
    matplotlibcpp::show();
}

void RRTSharp::initSearchSpace() noexcept
{
    m_numX = static_cast<int>(std::ceil((m_maxX - m_minX) / m_resolution));
    m_numY = static_cast<int>(std::ceil((m_maxY - m_minY) / m_resolution));

    m_numGrids = m_numX * m_numY;

    m_searchMap.resize(m_numGrids, false);
}

void RRTSharp::initObjects(const bool run) noexcept
{
    if (run == false)
    {
        // Object 1
        double minX = -80.0;
        double maxX = -30.0;
        double minY = -50.0;
        double maxY = -40.0;
        addObject(minX, maxX, minY, maxY);

        // Object 2
        minX = -10.0;
        maxX = +10.0;
        minY = -80.0;
        maxY = +20.0;
        addObject(minX, maxX, minY, maxY);

        // Object 3
        minX = -70.0;
        maxX = -40.0;
        minY = +10.0;
        maxY = +50.0;
        addObject(minX, maxX, minY, maxY);

        // Object 4
        minX = +30.0;
        maxX = +60.0;
        minY = +60.0;
        maxY = +90.0;
        addObject(minX, maxX, minY, maxY);

        // Object 5
        minX = +70.0;
        maxX = +90.0;
        minY = -70.0;
        maxY = -60.0;
        addObject(minX, maxX, minY, maxY);
    }

    if (run == true)
    {
        for (int indX = 0; indX < m_numX; ++indX)
        {
            for (int indY = 0; indY < m_numY; ++indY)
            {
                const int index = indX * m_numY + indY;
                if (m_observedMap.at(index) == 0)
                {
                    if (m_searchMap.at(index) == false)
                    {
                        m_searchMap[index] = true;
                        ++m_numOccupiedGrids;
                    }
                }
                else if (m_objectMap.at(index) > 2.0)
                {
                    if (m_searchMap.at(index) == false)
                    {
                        m_searchMap[index] = true;
                        ++m_numOccupiedGrids;
                    }
                }
                else
                {
                    // Do nothing
                }
            }
        }
    }
}

void RRTSharp::initVariables(const bool run) noexcept
{
    // A measure of the free space in the 2D grid map, number of un-occupied grids
    const double numFreeGrids = m_numGrids - m_numOccupiedGrids;
    m_gammaStar = std::sqrt(3.0 * numFreeGrids);

    if (run == false)
    {
        // Initialize start state
        m_startState = State(-90.0, -90.0, 10.0 * M_PI / 180.0);

        // Initialize goal state
        m_goalState = State(+90.0, +30.0, 300.0 * M_PI / 180.0);
        m_goalRaduis = 5.0;
    }
}

void RRTSharp::initGraph() noexcept
{
    const int id = 0;
    const int parentId = -1;
    const Cost g(0.0);
    const Cost lmc(0.0);

    m_nodeById.emplace(id, std::make_shared<Node>(m_goalState, id, parentId, g, lmc));
    m_nn->add(m_nodeById.at(id));
}

void RRTSharp::addObject(const double minX, const double maxX, const double minY, const double maxY) noexcept
{
    const int indexMinX = std::floor((minX - m_minX) / m_resolution);
    const int indexMaxX = std::floor((maxX - m_minX) / m_resolution);
    const int indexMinY = std::floor((minY - m_minY) / m_resolution);
    const int indexMaxY = std::floor((maxY - m_minY) / m_resolution);

    for (int indX = indexMinX; indX <= indexMaxX; ++indX)
    {
        for (int indY = indexMinY; indY <= indexMaxY; ++indY)
        {
            const int index = indX * m_numY + indY;
            m_searchMap[index] = true;
            ++m_numOccupiedGrids;
        }
    }
}

double RRTSharp::calculateRadius() const noexcept
{
    return m_gammaStar * std::sqrt(std::log(m_numExpandedGrids + 1) / m_numExpandedGrids) * m_resolution;
}

void RRTSharp::showSearchSpace() noexcept
{
    std::vector<std::uint8_t> searchMap(m_searchMap.size(), static_cast<std::uint8_t>(0));
    for (int indX = 0; indX < m_numX; ++indX)
    {
        for (int indY = 0; indY < m_numY; ++indY)
        {
            const int index = indX * m_numY + indY;
            const int indexT = indY * m_numX + indX;

            // Object occupied
            if (m_searchMap.at(index) == true)
            {
                searchMap[indexT] = static_cast<std::uint8_t>(1);
            }
        }
    }

//    matplotlibcpp::plot({m_startState.point().x()}, {m_startState.point().y()}, "o");

    std::map<std::string, std::string> keywords;
    keywords.emplace("origin", "lower");

    matplotlibcpp::imshow(searchMap.data(), m_numX, m_numY, 1, keywords);
    matplotlibcpp::show();
}

void RRTSharp::plotSearchSpace() noexcept
{
    std::map<std::string, std::string> keywords;
    keywords.emplace("color", "black");
    keywords.emplace("marker", "s");

    std::vector<float> objectX;
    std::vector<float> objectY;
    for (int indX = 0; indX < m_numX; ++indX)
    {
        objectX.clear();
        objectY.clear();
        for (int indY = 0; indY < m_numY; ++indY)
        {
            const int index = indX * m_numY + indY;

            // Object occupied
            if (m_searchMap.at(index) == true)
            {
                const double poseX = indX * m_resolution + m_minX;
                const double poseY = indY * m_resolution + m_minY;

                objectX.push_back(poseX);
                objectY.push_back(poseY);
            }
        }

        if (objectX.empty() == true)
        {
            continue;
        }

        matplotlibcpp::scatter(objectX, objectY, 10.0, keywords);
    }
}

void RRTSharp::plotStartState() noexcept
{
    std::map<std::string, std::string> keywords;
    keywords.emplace("color", "green");
    keywords.emplace("marker", "o");

    matplotlibcpp::plot({m_startState.point().x()}, {m_startState.point().y()}, keywords);
}

void RRTSharp::plotState(const State& state) noexcept
{
    std::map<std::string, std::string> keywords;
    keywords.emplace("color", "blue");
    keywords.emplace("marker", "s");

    matplotlibcpp::plot({state.point().x()}, {state.point().y()}, keywords);

}

void RRTSharp::plotPathFromIndeces(const std::vector<std::pair<int, int>>& pathIndeces) noexcept
{
    std::map<std::string, std::string> keywords;
    keywords.emplace("color", "blue");
//    keywords.emplace("marker", "-");

    std::vector<double> pathX;
    std::vector<double> pathY;

    for (const auto& xy : pathIndeces)
    {
        pathX.push_back(xy.first * m_resolution + m_minX);
        pathY.push_back(xy.second * m_resolution + m_minY);
    }

    matplotlibcpp::plot(pathX, pathY, keywords);
}

void RRTSharp::plotPath(const std::vector<State>& path) noexcept
{
    std::map<std::string, std::string> keywords;
    keywords.emplace("color", "blue");
//    keywords.emplace("marker", "-");

    std::vector<double> pathX;
    std::vector<double> pathY;

    for (const auto& state : path)
    {
        pathX.push_back(state.point().x());
        pathY.push_back(state.point().y());
    }

    matplotlibcpp::plot(pathX, pathY, keywords);
}

void RRTSharp::plotGoalState() noexcept
{
    std::map<std::string, std::string> keywords;
    keywords.emplace("color", "red");
    keywords.emplace("marker", "x");

    matplotlibcpp::plot({m_goalState.point().x()}, {m_goalState.point().y()}, keywords);
}

Key RRTSharp::key(const Node& node) const noexcept
{
    const double gMin = std::min(node.g().value(), node.lmc().value());
    const double f = gMin + heuristic(node);
    return {f, gMin};
}

double RRTSharp::heuristic(const Node& node) const noexcept
{
    return node.state().point().distanceTo(m_startState.point());
}

void RRTSharp::updateQueue(const std::shared_ptr<Node>& node)
{
    bool equalCost = false;
    if (std::abs(node->g().value() - node->lmc().value()) < 0.001)
    {
        equalCost = true;
    }

    const bool everAdded = (m_addPointers.count(node->m_handle.node_) > 0);
    const bool neverErased = (m_erasedPointers.count(node->m_handle.node_) == 0);

    if (neverErased == false)
    {
        return;
    }

    if (equalCost == false)
    {
        node->setKey(key(*node));

        if (everAdded == true)
        {
            m_queue.update(node->m_handle);
        }
        else if (everAdded == false)
        {
            node->m_handle = m_queue.push(node.get());
            m_addPointers.insert(node->m_handle.node_);
        }
    }
    else
    {
        if (everAdded == true)
        {
            m_queue.erase(node->m_handle);
            m_erasedPointers.insert(node->m_handle.node_);
        }
    }
}

State RRTSharp::sampleState() const noexcept
{
    static int scale = 1e4;
    static std::random_device randomDevice;
    static std::mt19937 randomNumberGenerator(randomDevice());
    static std::uniform_int_distribution<std::mt19937::result_type> uniformDistributionPoseX(1, (m_numX - 1) * scale);
    static std::uniform_int_distribution<std::mt19937::result_type> uniformDistributionPoseY(1, (m_numY - 1) * scale);
    static std::uniform_int_distribution<std::mt19937::result_type> uniformDistributionTheta(0, 360 * scale);

    const double poseX = static_cast<double>(uniformDistributionPoseX(randomNumberGenerator)) / static_cast<double>(scale) + m_minX;
    const double poseY = static_cast<double>(uniformDistributionPoseY(randomNumberGenerator)) / static_cast<double>(scale) + m_minY;
    const double theta = static_cast<double>(uniformDistributionTheta(randomNumberGenerator)) / static_cast<double>(scale) * M_PI / 180.0;

    return State(poseX, poseY, theta);
}
