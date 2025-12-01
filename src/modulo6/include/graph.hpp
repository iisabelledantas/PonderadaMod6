#ifndef GRAPH_HPP
#define GRAPH_HPP
#include <vector>
#include <string>
#include <utility>

struct Node {
    int row;     
    int col;      
    int g_cost;   // Custo do caminho até este nó (distância acumulada)
    
    Node* parent;
    
    Node() : row(0), col(0), g_cost(0), parent(nullptr) {}
    
    Node(int r, int c) : row(r), col(c), g_cost(0), parent(nullptr) {}
    
    Node(int r, int c, int g) : row(r), col(c), g_cost(g), parent(nullptr) {}
    
    bool operator==(const Node& other) const {
        return row == other.row && col == other.col;
    }
    
    bool operator!=(const Node& other) const {
        return !(*this == other);
    }
};

class Graph {
public:

    explicit Graph(const std::vector<std::vector<int>>& map);
    
    std::vector<Node> getNeighbors(const Node& node) const;
    
    bool isValidCell(int row, int col) const;
    
    int getRows() const { return rows_; }

    int getCols() const { return cols_; }

    const std::vector<std::vector<int>>& getMap() const { return map_; }
    
    int getEdgeCost() const { return 1; }

private:
    std::vector<std::vector<int>> map_;  
    int rows_;                            
    int cols_;                           

    const std::vector<std::pair<int, int>> directions_ = {
        {-1, 0},  // cima
        {1, 0},   // baixo
        {0, -1},  // esquerda
        {0, 1}    // direita
    };
};

std::vector<Node> reconstructPath(Node* goal);

std::vector<std::string> pathToDirections(const std::vector<Node>& path);

#endif 