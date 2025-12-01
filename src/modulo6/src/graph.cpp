#include "graph.hpp"
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

Graph::Graph(const std::vector<std::vector<int>>& map) : map_(map) {
    rows_ = static_cast<int>(map.size());
    cols_ = rows_ > 0 ? static_cast<int>(map[0].size()) : 0;
}

bool Graph::isValidCell(int row, int col) const {
    if (row < 0 || row >= rows_ || col < 0 || col >= cols_) {
        return false;
    }
    
    // Valores válidos: 0 (livre), 2 (robô), 3 (alvo)
    return map_[row][col] != 1;
}

std::vector<Node> Graph::getNeighbors(const Node& node) const {
    std::vector<Node> neighbors;

    for (const auto& dir : directions_) {
        int new_row = node.row + dir.first;   
        int new_col = node.col + dir.second;  
    
        if (isValidCell(new_row, new_col)) {
            neighbors.emplace_back(new_row, new_col);
        }
    }
    
    return neighbors;
}

std::vector<Node> reconstructPath(Node* goal) {
    std::vector<Node> path;
    
    // Percorrer de trás para frente usando os ponteiros parent
    Node* current = goal;
    while (current != nullptr) {
        path.push_back(*current);
        current = current->parent;
    }
    
    // Inverter o caminho para ir do início ao fim
    std::reverse(path.begin(), path.end());
    
    return path;
}

std::vector<std::string> pathToDirections(const std::vector<Node>& path) {
    std::vector<std::string> directions;
    
    for (size_t i = 1; i < path.size(); i++) {
        int row_diff = path[i].row - path[i-1].row;
        int col_diff = path[i].col - path[i-1].col;
        
        // Determinar a direção baseada na diferença de posições
        if (row_diff == -1 && col_diff == 0) {
            directions.push_back("up");      // Moveu para cima
        } else if (row_diff == 1 && col_diff == 0) {
            directions.push_back("down");    // Moveu para baixo
        } else if (row_diff == 0 && col_diff == -1) {
            directions.push_back("left");    // Moveu para esquerda
        } else if (row_diff == 0 && col_diff == 1) {
            directions.push_back("right");   // Moveu para direita
        }
    }
    
    return directions;
}