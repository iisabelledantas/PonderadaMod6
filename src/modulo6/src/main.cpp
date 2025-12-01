#include <rclcpp/rclcpp.hpp>
#include "ros_bridge.hpp"
#include "graph.hpp"
#include "navigation.hpp"
#include "mapping.hpp"
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>

void printMenu() {
    std::cout << "\n";
    std::cout << "╔══════════════════════════════════════════════════════╗\n";
    std::cout << "║     DESAFIO DO LABIRINTO - MENU PRINCIPAL           ║\n";
    std::cout << "╠══════════════════════════════════════════════════════╣\n";
    std::cout << "║                                                      ║\n";
    std::cout << "║  [1] Parte 1: Navegação com Dijkstra                 ║\n";
    std::cout << "║                                                      ║\n";
    std::cout << "║  [2] Parte 2: Exploração e Mapeamento (DFS)          ║\n";
    std::cout << "║                                                      ║\n";
    std::cout << "║  [3] Modo Completo (Parte 1 + Parte 2)               ║\n";
    std::cout << "║                                                      ║\n";
    std::cout << "║  [0] Sair                                            ║\n";
    std::cout << "║                                                      ║\n";
    std::cout << "╚══════════════════════════════════════════════════════╝\n";
    std::cout << "\nEscolha uma opção: ";
}

void executePart1(rclcpp::Node::SharedPtr node, ROSBridge& ros_bridge) {
    RCLCPP_INFO(node->get_logger(), "\n╔════════════════════════════════════════╗");
    RCLCPP_INFO(node->get_logger(), "║  PARTE 1: NAVEGAÇÃO COM DIJKSTRA       ║");
    RCLCPP_INFO(node->get_logger(), "╚════════════════════════════════════════╝\n");
    
    try {
        RCLCPP_INFO(node->get_logger(), "[1/4] Obtendo mapa do labirinto...");
        auto map = ros_bridge.getMap();
        
        if (map.empty()) {
            RCLCPP_ERROR(node->get_logger(), "Falha ao obter mapa!");
            return;
        }
        
        std::cout << "\n📊 Mapa obtido: " << map.size() << "x" << map[0].size() << "\n\n";
        for (const auto& row : map) {
            for (int cell : row) {
                if (cell == 0) std::cout << "· ";
                else if (cell == 1) std::cout << "█ ";
                else if (cell == 2) std::cout << "🤖";
                else if (cell == 3) std::cout << "🎯";
            }
            std::cout << "\n";
        }
        
        RCLCPP_INFO(node->get_logger(), "\n[2/4] Identificando posições...");
        auto robot_pos = ros_bridge.findRobotPosition(map);
        auto target_pos = ros_bridge.findTargetPosition(map);
        
        if (robot_pos.first == -1 || target_pos.first == -1) {
            RCLCPP_ERROR(node->get_logger(), "Não foi possível localizar robô ou alvo!");
            return;
        }
        
        std::cout << "🤖 Robô: (" << robot_pos.first << ", " << robot_pos.second << ")\n";
        std::cout << "🎯 Alvo: (" << target_pos.first << ", " << target_pos.second << ")\n";
        
        RCLCPP_INFO(node->get_logger(), "\n[3/4] Executando algoritmo de Dijkstra...\n");
        Graph graph(map);
        Navigation nav(graph);
        
        Node start(robot_pos.first, robot_pos.second);
        Node goal(target_pos.first, target_pos.second);
        
        auto path_dijkstra = nav.dijkstra(start, goal);
        
        if (path_dijkstra.empty()) {
            RCLCPP_ERROR(node->get_logger(), "Dijkstra não encontrou caminho!");
            return;
        }
        
        std::cout << "\n╔═══════════════════════════════════════╗\n";
        std::cout << "║     RESULTADO DO DIJKSTRA             ║\n";
        std::cout << "╠═══════════════════════════════════════╣\n";
        std::cout << "║ Caminho: " << std::setw(3) << path_dijkstra.size() << " passos            ║\n";
        std::cout << "║ Algoritmo: Dijkstra (Otimizado)      ║\n";
        std::cout << "╚═══════════════════════════════════════╝\n";
        
        RCLCPP_INFO(node->get_logger(), "\n[4/4] Executando navegação com Dijkstra...\n");
        
        auto directions = pathToDirections(path_dijkstra);
        
        for (size_t i = 0; i < directions.size(); i++) {
            std::cout << "▶ Movimento " << (i+1) << "/" << directions.size() 
                     << ": " << directions[i] << "... ";
            
            bool success = ros_bridge.moveRobot(directions[i]);
            
            if (success) {
                std::cout << "✓\n";
                std::this_thread::sleep_for(std::chrono::milliseconds(300));
            } else {
                std::cout << "✗\n";
                RCLCPP_ERROR(node->get_logger(), "Movimento falhou!");
                break;
            }
        }
        
        RCLCPP_INFO(node->get_logger(), "\n✅ PARTE 1 CONCLUÍDA COM SUCESSO!\n");
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Erro: %s", e.what());
    }
}

void executePart2(rclcpp::Node::SharedPtr node, ROSBridge& ros_bridge) {
    RCLCPP_INFO(node->get_logger(), "\n╔════════════════════════════════════════╗");
    RCLCPP_INFO(node->get_logger(), "║  PARTE 2: EXPLORAÇÃO E MAPEAMENTO      ║");
    RCLCPP_INFO(node->get_logger(), "╚════════════════════════════════════════╝\n");
    
    try {
        // [1/5] Resetar ambiente
        RCLCPP_INFO(node->get_logger(), "[1/5] Resetando ambiente...");
        ros_bridge.resetEnvironment();
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // [2/5] Obter informações do mapa
        RCLCPP_INFO(node->get_logger(), "\n[2/5] Obtendo informações do mapa...\n");
        auto real_map = ros_bridge.getMap();

        if (real_map.empty()) {
            RCLCPP_ERROR(node->get_logger(), "Falha ao obter mapa!");
            return;
        }

        auto initial_robot_pos = ros_bridge.findRobotPosition(real_map);
        auto target_pos = ros_bridge.findTargetPosition(real_map);

        if (initial_robot_pos.first == -1 || target_pos.first == -1) {
            RCLCPP_ERROR(node->get_logger(), "Robô ou alvo não encontrado!");
            return;
        }

        int map_rows = real_map.size();
        int map_cols = real_map[0].size();

        std::cout << "\n📏 Dimensões: " << map_rows << "x" << map_cols << "\n";
        std::cout << "🤖 Robô inicial: (" << initial_robot_pos.first << ", " << initial_robot_pos.second << ")\n";
        std::cout << "🎯 Alvo: (" << target_pos.first << ", " << target_pos.second << ")\n\n";

        auto original_target_pos = target_pos;

        RCLCPP_INFO(node->get_logger(), "\n[3/5] Explorando labirinto com DFS + Backtracking...\n");
        
        Mapper mapper(ros_bridge, map_rows, map_cols);
        mapper.setInitialPosition(initial_robot_pos.first, initial_robot_pos.second);
        
        auto explored_map = mapper.exploreAndMap("dfs");
        
        RCLCPP_INFO(node->get_logger(), "\n[4/5] Exportando mapa explorado...\n");
        
        for (int i = 0; i < map_rows; i++) {
            for (int j = 0; j < map_cols; j++) {
                if (explored_map[i][j] == ROBOT) {
                    explored_map[i][j] = FREE;
                }
                if (explored_map[i][j] == TARGET) {
                    explored_map[i][j] = FREE;
                }
            }
        }

        auto final_real_map = ros_bridge.getMap();
        auto actual_robot_pos = ros_bridge.findRobotPosition(final_real_map);
        
        if (actual_robot_pos.first != -1) {
            explored_map[actual_robot_pos.first][actual_robot_pos.second] = FREE;
            std::cout << "📍 Robô final: (" << actual_robot_pos.first << ", " << actual_robot_pos.second << ")\n";
        }
        
        if (original_target_pos.first != -1) {
            explored_map[original_target_pos.first][original_target_pos.second] = FREE;
            std::cout << "🎯 Alvo original: (" << original_target_pos.first << ", " << original_target_pos.second << ")\n\n";
        }
        mapper.exportMapToCSV("explored_map.csv");
        
        int free_count = 0, unknown_count = 0, wall_count = 0;
        for (const auto& row : explored_map) {
            for (int cell : row) {
                if (cell == FREE) free_count++;
                else if (cell == UNKNOWN) unknown_count++;
                else if (cell == WALL) wall_count++;
            }
        }
        
        float exploration_pct = (float)(free_count + wall_count) / (map_rows * map_cols) * 100;
        
        std::cout << "\n📊 ESTATÍSTICAS DA EXPLORAÇÃO:\n";
        std::cout << "   Células livres: " << free_count << "\n";
        std::cout << "   Células desconhecidas: " << unknown_count << "\n";
        std::cout << "   Paredes: " << wall_count << "\n";
        std::cout << "   Exploração: " << std::fixed << std::setprecision(1) << exploration_pct << "%\n\n";
        
        RCLCPP_INFO(node->get_logger(), "\n[5/5] Verificando resultado...\n");
        
        bool target_reached = false;
        if (actual_robot_pos.first == original_target_pos.first && 
            actual_robot_pos.second == original_target_pos.second) {
            target_reached = true;
        }
        
        if (!target_reached && actual_robot_pos.first != -1 && original_target_pos.first != -1) {
            int dist = std::abs(actual_robot_pos.first - original_target_pos.first) +
                      std::abs(actual_robot_pos.second - original_target_pos.second);
            if (dist <= 1) {
                target_reached = true;
            }
        }
        
        std::cout << "\n";
        if (target_reached) {
            std::cout << "╔═══════════════════════════════════════════════╗\n";
            std::cout << "║                                               ║\n";
            std::cout << "║   🎉 🏆 ALVO ALCANÇADO COM SUCESSO! 🏆 🎉    ║\n";
            std::cout << "║                                               ║\n";
            std::cout << "╚═══════════════════════════════════════════════╝\n";
            RCLCPP_INFO(node->get_logger(), "\n✅ PARTE 2 CONCLUÍDA COM SUCESSO!\n");
        } else {
            std::cout << "⚠️  Exploração completa, mas alvo não foi alcançado\n";
            std::cout << "   Posição final do robô: (" << actual_robot_pos.first << ", " << actual_robot_pos.second << ")\n";
            std::cout << "   Posição do alvo: (" << original_target_pos.first << ", " << original_target_pos.second << ")\n";
            
            int final_dist = std::abs(actual_robot_pos.first - original_target_pos.first) +
                            std::abs(actual_robot_pos.second - original_target_pos.second);
            std::cout << "   Distância Manhattan: " << final_dist << " células\n";
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Erro: %s", e.what());
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("maze_solver");
    
    std::cout << "\n";
    std::cout << "╔══════════════════════════════════════════════════════╗\n";
    std::cout << "║                Ponderada                             ║\n";
    std::cout << "╚══════════════════════════════════════════════════════╝\n";
    
    try {
        ROSBridge ros_bridge(node);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        while (true) {
            printMenu();
            
            int choice;
            std::cin >> choice;
            
            switch (choice) {
                case 1:
                    executePart1(node, ros_bridge);
                    break;
                    
                case 2:
                    executePart2(node, ros_bridge);
                    break;
                    
                case 3:
                    executePart1(node, ros_bridge);
                    std::cout << "\nPressione Enter para continuar para a Parte 2...";
                    std::cin.ignore();
                    std::cin.get();
                    executePart2(node, ros_bridge);
                    break;
                    
                case 0:
                    RCLCPP_INFO(node->get_logger(), "Encerrando programa...");
                    rclcpp::shutdown();
                    return 0;
                    
                default:
                    std::cout << "❌ Opção inválida!\n";
            }
            
            std::cout << "\nPressione Enter para voltar ao menu...";
            std::cin.ignore();
            std::cin.get();
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Erro fatal: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}