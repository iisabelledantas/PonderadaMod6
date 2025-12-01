#include "navigation.hpp"
#include <queue>
#include <algorithm>
#include <iostream>
#include <map>
#include <set>

Navigation::Navigation(const Graph& graph) : graph_(graph) {}

bool Navigation::isVisited(const std::vector<std::vector<bool>>& visited, const Node& node) const {
    return visited[node.row][node.col];
}

void Navigation::markVisited(std::vector<std::vector<bool>>& visited, const Node& node) {
    visited[node.row][node.col] = true;
}

std::vector<Node> Navigation::dijkstra(const Node& start, const Node& goal) {
    std::cout << "\n=== Executando Dijkstra ===\n";
    std::cout << "Início: (" << start.row << ", " << start.col << ")\n";
    std::cout << "Alvo: (" << goal.row << ", " << goal.col << ")\n";

    std::vector<std::vector<bool>> visited(
        graph_.getRows(), 
        std::vector<bool>(graph_.getCols(), false)
    );

    std::map<std::pair<int, int>, Node*> nodes;
    auto compare = [](const std::pair<int, Node*>& a, const std::pair<int, Node*>& b) {
        return a.first > b.first;  
    };
    std::priority_queue<std::pair<int, Node*>, 
                       std::vector<std::pair<int, Node*>>, 
                       decltype(compare)> frontier(compare);

    Node* start_node = new Node(start);
    start_node->g_cost = 0;
    nodes[{start.row, start.col}] = start_node;
    frontier.push({0, start_node});

    int nodes_explored = 0;
    Node* goal_node = nullptr;

    while (!frontier.empty()) {
        auto [current_cost, current] = frontier.top();
        frontier.pop();

        if (isVisited(visited, *current)) {
            continue;
        }

        markVisited(visited, *current);
        nodes_explored++;

        if (current->row == goal.row && current->col == goal.col) {
            goal_node = current;
            std::cout << "🎯 Objetivo alcançado!\n";
            break;
        }

        std::vector<Node> neighbors = graph_.getNeighbors(*current);
        for (const auto& neighbor : neighbors) {
            if (isVisited(visited, neighbor)) {
                continue;
            }

            int new_cost = current->g_cost + 1;
            auto key = std::make_pair(neighbor.row, neighbor.col);

            if (nodes.find(key) == nodes.end() || new_cost < nodes[key]->g_cost) {
                Node* neighbor_node;
                
                if (nodes.find(key) == nodes.end()) {
                    neighbor_node = new Node(neighbor);
                    nodes[key] = neighbor_node;
                } else {
                    neighbor_node = nodes[key];
                }
                neighbor_node->g_cost = new_cost;
                neighbor_node->parent = current;
                frontier.push({new_cost, neighbor_node});
            }
        }
    }

    std::vector<Node> path;
    
    if (goal_node != nullptr) {
        std::cout << "✅ Caminho encontrado!\n";
        std::cout << "📊 Nós explorados: " << nodes_explored << "\n";
        std::cout << "📏 Custo total: " << goal_node->g_cost << " passos\n";
        path = reconstructPath(goal_node);
        std::cout << "🛣️  Tamanho do caminho: " << path.size() << " células\n";
    } else {
        std::cout << "❌ Nenhum caminho encontrado!\n";
        std::cout << "📊 Nós explorados: " << nodes_explored << "\n";
    }

    for (auto& p : nodes) {
        delete p.second;
    }

    return path;
}
