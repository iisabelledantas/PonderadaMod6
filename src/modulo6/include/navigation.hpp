#ifndef NAVIGATION_HPP
#define NAVIGATION_HPP

#include "graph.hpp"
#include <vector>
#include <string>


class Navigation {
public:
    
    explicit Navigation(const Graph& graph);

    std::vector<Node> dijkstra(const Node& start, const Node& goal);

private:
    
    const Graph& graph_;

    bool isVisited(const std::vector<std::vector<bool>>& visited, const Node& node) const;

    void markVisited(std::vector<std::vector<bool>>& visited, const Node& node);
};

#endif