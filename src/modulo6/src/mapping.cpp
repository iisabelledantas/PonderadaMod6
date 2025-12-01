#include "mapping.hpp"
#include "navigation.hpp"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <queue>
#include <set>
#include <map>
#include <stack>
#include <filesystem>
#include <thread>
#include <chrono>
#include <cmath>
#include <climits>
#include <random>

Mapper::Mapper(ROSBridge& ros_bridge, int max_rows, int max_cols)
    : ros_bridge_(ros_bridge), max_rows_(max_rows), max_cols_(max_cols),
      current_row_(max_rows/2), current_col_(max_cols/2), current_direction_("north") {
    
    internal_map_.resize(max_rows_, std::vector<int>(max_cols_, UNKNOWN));
    setCellValue(current_row_, current_col_, FREE);
    
    std::cout << "🗺️  Mapper SLAM inicializado. Mapa: " << max_rows_ << "x" << max_cols_ << "\n";
    std::cout << "📍 Posição inicial: (" << current_row_ << ", " << current_col_ << ")\n";
}


std::vector<std::vector<int>> Mapper::exploreAndMap(const std::string& strategy) {
    (void)strategy;
    
    std::cout << "\n╔═══════════════════════════════════════════════════════╗\n";
    std::cout << "║  🤖 EXPLORAÇÃO COM DFS + BACKTRACKING                 ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════╝\n\n";
    
    exploreDFSWithBacktracking();
    
    std::cout << "\n✅ MAPEAMENTO CONCLUÍDO\n";
    printMap();
    
    return internal_map_;
}





void Mapper::exploreDFSWithBacktracking() {
    std::cout << "🔍 Iniciando exploração DFS com backtracking...\n\n";
    
    std::set<std::pair<int, int>> visited;
    std::stack<std::pair<int, int>> path_stack;
    
    
    syncPositionWithServer();
    visited.insert({current_row_, current_col_});
    path_stack.push({current_row_, current_col_});
    
    int total_moves = 0;
    const int MAX_MOVES = 5000;
    int iterations = 0;
    
    std::cout << "📍 Posição inicial: (" << current_row_ << ", " << current_col_ << ")\n";
    std::cout << "🎯 Objetivo: Encontrar e alcançar o TARGET\n\n";
    
    while (total_moves < MAX_MOVES && iterations < MAX_MOVES) {
        iterations++;
        
        
        
        
        updateMapFromSensorsProper();
        
        
        
        
        if (hasFoundTarget()) {
            auto target_pos = getTargetPosition();
            
            
            int dist = std::abs(target_pos.first - current_row_) + 
                      std::abs(target_pos.second - current_col_);
            
            if (dist == 1) {
                std::cout << "\n🎯 ✅ ALVO ADJACENTE DETECTADO em (" << target_pos.first 
                         << ", " << target_pos.second << ")!\n";
                std::cout << "🚀 Movendo para o alvo...\n\n";
                
                
                std::string direction = calculateDirectionToCell(target_pos.first, target_pos.second);
                if (!direction.empty() && moveAndUpdate(direction)) {
                    std::cout << "\n🎉 🏆 ALVO ALCANÇADO COM SUCESSO!\n";
                    break;
                }
            }
        }
        
        
        
        
        std::string next_direction = decideNextMoveWithBacktracking(visited, path_stack);
        
        if (next_direction.empty()) {
            std::cout << "\n⚠️  Exploração completa - nenhum movimento possível!\n";
            break;
        }
        
        
        auto offset = getDirectionOffset(next_direction);
        int next_row = current_row_ + offset.first;
        int next_col = current_col_ + offset.second;
        
        bool is_backtrack = false;
        if (path_stack.size() > 1) {
            std::stack<std::pair<int, int>> temp_stack = path_stack;
            temp_stack.pop(); 
            auto prev_pos = temp_stack.top();
            if (next_row == prev_pos.first && next_col == prev_pos.second) {
                is_backtrack = true;
            }
        }
        
        
        
        
        if (moveAndUpdate(next_direction)) {
            visited.insert({current_row_, current_col_});
            
            if (is_backtrack) {
                path_stack.pop(); 
                std::cout << "  ↩️  Backtracking\n";
            } else {
                path_stack.push({current_row_, current_col_}); 
                std::cout << "  ✓ Explorando nova célula\n";
            }
            
            total_moves++;
        } else {
            
            setCellValue(next_row, next_col, WALL);
        }
        
        
        if (total_moves % 10 == 0) {
            std::cout << "\r📊 Progresso: " << visited.size() << " células visitadas" << std::flush;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
    
    std::cout << "\n\n✅ Exploração concluída!\n";
    std::cout << "   Movimentos totais: " << total_moves << "\n";
    std::cout << "   Células visitadas: " << visited.size() << "\n";
    std::cout << "   Cobertura: " << std::fixed << std::setprecision(1) 
             << calculateCoverage() << "%\n";
}





std::string Mapper::decideNextMoveWithBacktracking(
    const std::set<std::pair<int, int>>& visited,
    std::stack<std::pair<int, int>>& path_stack) {
    
    
    std::vector<std::pair<std::string, std::pair<int, int>>> directions = {
        {"up", {-1, 0}},
        {"right", {0, 1}},
        {"down", {1, 0}},
        {"left", {0, -1}}
    };
    
    
    if (hasFoundTarget()) {
        auto target_pos = getTargetPosition();
        for (const auto& [dir_name, offset] : directions) {
            int nr = current_row_ + offset.first;
            int nc = current_col_ + offset.second;
            
            if (nr == target_pos.first && nc == target_pos.second) {
                if (getCellValue(nr, nc) == TARGET) {
                    return dir_name;
                }
            }
        }
    }
    
    
    for (const auto& [dir_name, offset] : directions) {
        int nr = current_row_ + offset.first;
        int nc = current_col_ + offset.second;
        
        
        if (nr < 0 || nr >= max_rows_ || nc < 0 || nc >= max_cols_) {
            continue;
        }
        
        
        if (visited.find({nr, nc}) == visited.end()) {
            int cell_value = getCellValue(nr, nc);
            if (cell_value == FREE || cell_value == UNKNOWN || cell_value == TARGET) {
                return dir_name;
            }
        }
    }
    
    
    if (path_stack.size() > 1) {
        std::stack<std::pair<int, int>> temp_stack = path_stack;
        temp_stack.pop(); 
        auto prev_pos = temp_stack.top();
        
        int diff_row = prev_pos.first - current_row_;
        int diff_col = prev_pos.second - current_col_;
        
        for (const auto& [dir_name, offset] : directions) {
            if (offset.first == diff_row && offset.second == diff_col) {
                return dir_name;
            }
        }
    }
    
    return ""; 
}





std::string Mapper::calculateDirectionToCell(int target_row, int target_col) const {
    int diff_row = target_row - current_row_;
    int diff_col = target_col - current_col_;
    
    if (diff_row == -1 && diff_col == 0) return "up";
    if (diff_row == 1 && diff_col == 0) return "down";
    if (diff_row == 0 && diff_col == -1) return "left";
    if (diff_row == 0 && diff_col == 1) return "right";
    
    return "";
}





std::pair<int, int> Mapper::getDirectionOffset(const std::string& direction) const {
    if (direction == "up") return {-1, 0};
    if (direction == "down") return {1, 0};
    if (direction == "left") return {0, -1};
    if (direction == "right") return {0, 1};
    return {0, 0};
}





void Mapper::updateMapFromSensorsProper() {
    
    setCellValue(current_row_, current_col_, FREE);
    
    
    auto sensors = ros_bridge_.getSensorData();
    
    
    std::map<std::string, std::pair<int, int>> sensor_directions = {
        {"up", {-1, 0}},
        {"down", {1, 0}},
        {"left", {0, -1}},
        {"right", {0, 1}}
    };
    
    std::map<std::string, std::string> sensor_values = {
        {"up", sensors.up},
        {"down", sensors.down},
        {"left", sensors.left},
        {"right", sensors.right}
    };
    
    int updates = 0;
    
    
    for (const auto& [dir_name, offset] : sensor_directions) {
        int adj_row = current_row_ + offset.first;
        int adj_col = current_col_ + offset.second;
        
        
        if (adj_row < 0 || adj_row >= max_rows_ || 
            adj_col < 0 || adj_col >= max_cols_) {
            continue;
        }
        
        
        std::string cell_type = sensor_values[dir_name];
        
        if (cell_type.empty() || cell_type == "unknown" || cell_type == "u") {
            continue;
        }
        
        char type_char = cell_type[0];
        
        
        if (type_char == 'f' || type_char == '0') {
            
            if (getCellValue(adj_row, adj_col) != TARGET) {
                setCellValue(adj_row, adj_col, FREE);
                updates++;
            }
        } else if (type_char == 'w' || type_char == 'b' || type_char == '1') {
            
            setCellValue(adj_row, adj_col, WALL);
            updates++;
        } else if (type_char == 't' || type_char == 'g' || type_char == '3') {
            
            setCellValue(adj_row, adj_col, TARGET);
            updates++;
            std::cout << "\n🎯 ⚡ ALVO DETECTADO em (" << adj_row << "," 
                     << adj_col << ") via SENSOR!\n";
        } else if (type_char == 'r' || type_char == '2') {
            
            if (getCellValue(adj_row, adj_col) != TARGET) {
                setCellValue(adj_row, adj_col, FREE);
                updates++;
            }
        }
    }
}





bool Mapper::moveAndUpdate(const std::string& direction) {
    bool success = ros_bridge_.moveRobot(direction);
    
    if (success) {
        
        if (direction == "up") current_row_--;
        else if (direction == "down") current_row_++;
        else if (direction == "left") current_col_--;
        else if (direction == "right") current_col_++;
        
        updateDirection(direction);
        setCellValue(current_row_, current_col_, FREE);
        
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        updateMapFromSensorsProper();
    }
    
    return success;
}





bool Mapper::navigateToTarget() {
    auto target_pos = getTargetPosition();
    
    if (target_pos.first == -1) {
        return false;
    }
    
    std::cout << "🗺️  Planejando rota Dijkstra de (" << current_row_ << "," 
             << current_col_ << ") → (" << target_pos.first << "," 
             << target_pos.second << ")\n";
    
    return navigateToCell(target_pos.first, target_pos.second);
}





bool Mapper::navigateToCell(int target_row, int target_col) {
    if (current_row_ == target_row && current_col_ == target_col) {
        return true;
    }
    
    
    auto temp_map = internal_map_;
    for (int i = 0; i < max_rows_; i++) {
        for (int j = 0; j < max_cols_; j++) {
            if (temp_map[i][j] == UNKNOWN) {
                temp_map[i][j] = WALL;
            }
            if (temp_map[i][j] == ROBOT) {
                temp_map[i][j] = FREE;
            }
            if (temp_map[i][j] == TARGET) {
                temp_map[i][j] = FREE;
            }
        }
    }
    
    temp_map[current_row_][current_col_] = FREE;
    temp_map[target_row][target_col] = FREE;
    
    
    Graph graph(temp_map);
    Navigation nav(graph);
    
    Node start(current_row_, current_col_);
    Node goal(target_row, target_col);
    
    auto path = nav.dijkstra(start, goal);
    
    if (path.empty() || path.size() == 1) {
        return false;
    }
    
    
    auto directions = pathToDirections(path);
    
    std::cout << "   🚶 Executando " << directions.size() << " movimentos...\n";
    
    
    for (size_t i = 0; i < directions.size(); i++) {
        if (!moveAndUpdate(directions[i])) {
            std::cout << "   ❌ Movimento " << (i+1) << " bloqueado\n";
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    return true;
}





void Mapper::updateMapFromSensors(const cg_interfaces::msg::RobotSensors& sensors) {
    (void)sensors;
    updateMapFromSensorsProper();
}

void Mapper::syncPositionWithServer() {
    auto real_map = ros_bridge_.getMap();
    auto real_pos = ros_bridge_.findRobotPosition(real_map);
    
    if (real_pos.first != -1) {
        current_row_ = real_pos.first;
        current_col_ = real_pos.second;
    }
}

bool Mapper::hasFoundTarget() const {
    for (int i = 0; i < max_rows_; i++) {
        for (int j = 0; j < max_cols_; j++) {
            if (internal_map_[i][j] == TARGET) {
                return true;
            }
        }
    }
    return false;
}

std::pair<int, int> Mapper::getTargetPosition() const {
    for (int i = 0; i < max_rows_; i++) {
        for (int j = 0; j < max_cols_; j++) {
            if (internal_map_[i][j] == TARGET) {
                return {i, j};
            }
        }
    }
    return {-1, -1};
}

float Mapper::calculateCoverage() const {
    int explored = 0;
    int total = max_rows_ * max_cols_;
    
    for (int i = 0; i < max_rows_; i++) {
        for (int j = 0; j < max_cols_; j++) {
            if (internal_map_[i][j] != UNKNOWN) {
                explored++;
            }
        }
    }
    
    return (float)explored / total * 100.0f;
}

void Mapper::setCellValue(int row, int col, int value) {
    if (row >= 0 && row < max_rows_ && col >= 0 && col < max_cols_) {
        internal_map_[row][col] = value;
    }
}

int Mapper::getCellValue(int row, int col) const {
    if (row >= 0 && row < max_rows_ && col >= 0 && col < max_cols_) {
        return internal_map_[row][col];
    }
    return WALL;
}

bool Mapper::isNavigable(int row, int col) const {
    int value = getCellValue(row, col);
    return value == FREE || value == TARGET;
}

void Mapper::setInitialPosition(int row, int col) {
    current_row_ = row;
    current_col_ = col;
    setCellValue(row, col, FREE);
    std::cout << "📍 Posição inicial: (" << row << ", " << col << ")\n";
}

void Mapper::updateDirection(const std::string& movement) {
    if (movement == "up") current_direction_ = "north";
    else if (movement == "down") current_direction_ = "south";
    else if (movement == "left") current_direction_ = "west";
    else if (movement == "right") current_direction_ = "east";
}

bool Mapper::exportMapToCSV(const std::string& filename) {
    std::filesystem::create_directories("maps");
    std::string full_path = "maps/" + filename;
    std::ofstream file(full_path);
    
    if (!file.is_open()) {
        std::cerr << "Erro ao abrir arquivo: " << full_path << "\n";
        return false;
    }
    
    for (const auto& row : internal_map_) {
        for (size_t i = 0; i < row.size(); i++) {
            file << row[i];
            if (i < row.size() - 1) {
                file << ",";
            }
        }
        file << "\n";
    }
    
    file.close();
    std::cout << "✅ Mapa exportado para: " << full_path << "\n";
    return true;
}

void Mapper::printMap() const {
    std::cout << "\n=== MAPA EXPLORADO ===\n\n";
    
    for (int i = 0; i < max_rows_; i++) {
        for (int j = 0; j < max_cols_; j++) {
            int value = internal_map_[i][j];
            
            if (i == current_row_ && j == current_col_) {
                std::cout << "🤖";
            } else if (value == UNKNOWN) {
                std::cout << "? ";
            } else if (value == FREE) {
                std::cout << "· ";
            } else if (value == WALL) {
                std::cout << "█ ";
            } else if (value == TARGET) {
                std::cout << "🎯";
            }
        }
        std::cout << "\n";
    }
    std::cout << "\n";
}



