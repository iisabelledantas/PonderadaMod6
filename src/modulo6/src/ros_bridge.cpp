#include "ros_bridge.hpp"
#include <chrono>
#include <memory>
#include <string>
#include <sstream>
#include <thread>

using namespace std::chrono_literals;

ROSBridge::ROSBridge(rclcpp::Node::SharedPtr node) : node_(node) {

    get_map_client_ = node_->create_client<cg_interfaces::srv::GetMap>("/get_map");
    move_cmd_client_ = node_->create_client<cg_interfaces::srv::MoveCmd>("/move_command");
    reset_client_ = node_->create_client<cg_interfaces::srv::Reset>("/reset");
    
    sensor_sub_ = node_->create_subscription<cg_interfaces::msg::RobotSensors>(
        "/culling_games/robot_sensors", 10,
        std::bind(&ROSBridge::sensorCallback, this, std::placeholders::_1)
    );
    
    last_sensor_data_.up = "unknown";
    last_sensor_data_.down = "unknown";
    last_sensor_data_.left = "unknown";
    last_sensor_data_.right = "unknown";
    last_sensor_data_.up_left = "unknown";
    last_sensor_data_.up_right = "unknown";
    last_sensor_data_.down_left = "unknown";
    last_sensor_data_.down_right = "unknown";
    
    RCLCPP_INFO(node_->get_logger(), "ROSBridge inicializado com sucesso!");
}

std::vector<std::vector<int>> ROSBridge::getMap() {
    
    while (!get_map_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node_->get_logger(), "Interrompido enquanto aguardava o serviço GetMap");
            return {};
        }
        RCLCPP_INFO(node_->get_logger(), "Aguardando serviço GetMap...");
    }

    auto request = std::make_shared<cg_interfaces::srv::GetMap::Request>();

    auto future = get_map_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, future, 5s) == 
        rclcpp::FutureReturnCode::SUCCESS) {
        
        auto response = future.get();

        if (response->occupancy_grid_shape.size() < 2) {
            RCLCPP_ERROR(node_->get_logger(), "Formato de shape inválido!");
            return {};
        }
        
        int rows = response->occupancy_grid_shape[0];  
        int cols = response->occupancy_grid_shape[1];  
        
        RCLCPP_INFO(node_->get_logger(), "Mapa recebido: %dx%d células", rows, cols);

        if (response->occupancy_grid_flattened.size() != static_cast<size_t>(rows * cols)) {
            RCLCPP_ERROR(node_->get_logger(), 
                        "Tamanho inconsistente: esperado %d, recebido %zu",
                        rows * cols, 
                        response->occupancy_grid_flattened.size());
            return {};
        }
        
        std::vector<std::vector<int>> map_2d;
        map_2d.resize(rows, std::vector<int>(cols, 0));
        
        auto charToInt = [](const std::string& str) -> int {
            if (str.empty()) return 0;
            
            char c = str[0];
        
            switch (c) {
                case 'f':  
                case '0':
                    return 0;
                    
                case 'b':  
                case 'w':  
                case '1':
                    return 1;
                    
                case 'r':  
                case '2':
                    return 2;
                    
                case 't':  
                case 'g':  
                case '3':
                    return 3;
                    
                default:
                    
                    if (std::isdigit(c)) {
                        return c - '0';
                    }
                    return 0;              }
        };
        
        try {
            
            for (int i = 0; i < rows; i++) {
                for (int j = 0; j < cols; j++) {
                    int index = i * cols + j;
                    std::string cell_str = response->occupancy_grid_flattened[index];
                    
                    map_2d[i][j] = charToInt(cell_str);
                }
            }
            
            int count_free = 0, count_wall = 0, count_robot = 0, count_target = 0;
            for (const auto& row : map_2d) {
                for (int cell : row) {
                    if (cell == 0) count_free++;
                    else if (cell == 1) count_wall++;
                    else if (cell == 2) count_robot++;
                    else if (cell == 3) count_target++;
                }
            }
            
            RCLCPP_INFO(node_->get_logger(), "Mapa convertido com sucesso!");
            RCLCPP_INFO(node_->get_logger(), "  Células livres: %d", count_free);
            RCLCPP_INFO(node_->get_logger(), "  Paredes: %d", count_wall);
            RCLCPP_INFO(node_->get_logger(), "  Robôs: %d", count_robot);
            RCLCPP_INFO(node_->get_logger(), "  Alvos: %d", count_target);
            
            return map_2d;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Erro ao converter mapa: %s", e.what());
            return {};
        }
        
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Falha ao chamar serviço GetMap (timeout)");
        return {};
    }
}

bool ROSBridge::moveRobot(const std::string& direction) {
    if (!move_cmd_client_->wait_for_service(10s)) {
        RCLCPP_WARN(node_->get_logger(), "Serviço MoveCmd não está disponível");
        return false;
    }

    auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
    request->direction = direction;
    
    auto future = move_cmd_client_->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(node_, future, 2s) == 
        rclcpp::FutureReturnCode::SUCCESS) {
        
        auto response = future.get();
        
        if (response->success) {
            RCLCPP_INFO(node_->get_logger(), "Movimento '%s' executado com sucesso. "
                       "Robô: (%d,%d), Alvo: (%d,%d)", 
                       direction.c_str(),
                       response->robot_pos[0], response->robot_pos[1],
                       response->target_pos[0], response->target_pos[1]);
        } else {
            RCLCPP_WARN(node_->get_logger(), "Movimento '%s' bloqueado", 
                       direction.c_str());
        }
        
        return response->success;
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Falha ao chamar serviço MoveCmd");
        return false;
    }
}

bool ROSBridge::resetEnvironment() {
    if (!reset_client_->wait_for_service(1s)) {
        RCLCPP_WARN(node_->get_logger(), "Serviço Reset não está disponível");
        return false;
    }

    auto request = std::make_shared<cg_interfaces::srv::Reset::Request>();
    
    auto future = reset_client_->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(node_, future, 2s) == 
        rclcpp::FutureReturnCode::SUCCESS) {
        
        RCLCPP_INFO(node_->get_logger(), "Ambiente resetado com sucesso!");
        return true;
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Falha ao resetar ambiente");
        return false;
    }
}

cg_interfaces::msg::RobotSensors ROSBridge::getSensorData() {
    int attempts = 0;
    const int MAX_ATTEMPTS = 50;
    
    while (attempts < MAX_ATTEMPTS) {
        rclcpp::spin_some(node_);
        if (last_sensor_data_.up != "unknown") {
            return last_sensor_data_;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        attempts++;
    }
    
    RCLCPP_WARN(node_->get_logger(), "Timeout ao aguardar dados dos sensores!");
    return last_sensor_data_;
}

void ROSBridge::sensorCallback(const cg_interfaces::msg::RobotSensors::SharedPtr msg) {
    last_sensor_data_ = *msg;
    RCLCPP_DEBUG(node_->get_logger(), "Sensores atualizados!");
}

std::pair<int, int> ROSBridge::findRobotPosition(const std::vector<std::vector<int>>& map) {
    for (size_t i = 0; i < map.size(); i++) {
        for (size_t j = 0; j < map[i].size(); j++) {
            if (map[i][j] == 2) {  
                RCLCPP_INFO(node_->get_logger(), "Robô encontrado na posição (%zu, %zu)", i, j);
                return {static_cast<int>(i), static_cast<int>(j)};
            }
        }
    }
    
    RCLCPP_ERROR(node_->get_logger(), "Robô não encontrado no mapa!");
    return {-1, -1};  
}

std::pair<int, int> ROSBridge::findTargetPosition(const std::vector<std::vector<int>>& map) {

    for (size_t i = 0; i < map.size(); i++) {
        for (size_t j = 0; j < map[i].size(); j++) {
            if (map[i][j] == 3) {  
                RCLCPP_INFO(node_->get_logger(), "Alvo encontrado na posição (%zu, %zu)", i, j);
                return {static_cast<int>(i), static_cast<int>(j)};
            }
        }
    }
    
    RCLCPP_ERROR(node_->get_logger(), "Alvo não encontrado no mapa!");
    return {-1, -1};  
}