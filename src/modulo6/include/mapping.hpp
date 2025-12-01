#ifndef MAPPING_HPP
#define MAPPING_HPP

#include "ros_bridge.hpp"
#include "graph.hpp"
#include <vector>
#include <string>
#include <set>
#include <stack>

// ============================================================================
// CONSTANTES: Valores das células do mapa ocupacional
// ============================================================================
const int UNKNOWN = -1;  // Célula não explorada
const int FREE = 0;      // Célula livre (branca)
const int WALL = 1;      // Parede (preta)
const int ROBOT = 2;     // Robô (azul)
const int TARGET = 3;    // Alvo (vermelho)

/**
 * @brief Classe Mapper - Implementa SLAM baseado em Grid com DFS + Backtracking
 * 
 * Esta classe é responsável por:
 * 1. Construir incrementalmente um Occupancy Grid Map do ambiente
 * 2. Explorar o ambiente usando DFS com backtracking
 * 3. Detectar o alvo via sensores
 * 4. Navegar até o alvo usando movimento direto quando detectado
 * 
 * Estratégia:
 * - DFS com Backtracking: Explora profundamente e volta quando necessário
 * - Sensor Fusion: Atualiza mapa com leituras de sensores a cada movimento
 * - Detecção Adjacente: Quando alvo detectado adjacente, move diretamente
 */
class Mapper {
public:
    /**
     * @brief Construtor
     * @param ros_bridge Referência ao objeto de comunicação ROS2
     * @param max_rows Número máximo de linhas do mapa
     * @param max_cols Número máximo de colunas do mapa
     */
    Mapper(ROSBridge& ros_bridge, int max_rows, int max_cols);
    
    // ========================================================================
    // MÉTODOS PRINCIPAIS DE EXPLORAÇÃO
    // ========================================================================
    
    /**
     * @brief Método principal de exploração e mapeamento
     * @param strategy Nome da estratégia (ignorado, sempre usa DFS+Backtracking)
     * @return Mapa 2D construído
     */
    std::vector<std::vector<int>> exploreAndMap(const std::string& strategy);
    
    /**
     * @brief Exploração DFS com Backtracking (SLAM)
     * 
     * Algoritmo:
     * 1. Ler sensores e atualizar mapa ocupacional
     * 2. Se alvo detectado adjacente → mover diretamente
     * 3. Senão → explorar células não visitadas (DFS)
     * 4. Se sem opções → backtracking usando pilha
     * 5. Repetir até alvo alcançado ou exploração completa
     */
    void exploreDFSWithBacktracking();
    
    /**
     * @brief Wrapper para compatibilidade - chama exploreDFSWithBacktracking()
     */
    void exploreFrontierBased();
    
    /**
     * @brief Wrapper para compatibilidade - chama exploreDFSWithBacktracking()
     */
    void exploreBFS();
    
    // ========================================================================
    // ATUALIZAÇÃO DO MAPA (SENSOR FUSION)
    // ========================================================================
    
    /**
     * @brief Atualiza o mapa ocupacional com leituras dos sensores
     * 
     * Para cada direção (up, down, left, right):
     * - Lê sensor
     * - Atualiza célula adjacente no mapa:
     *   * 'f'/'0' → FREE
     *   * 'w'/'b'/'1' → WALL
     *   * 't'/'g'/'3' → TARGET
     */
    void updateMapFromSensorsProper();
    
    /**
     * @brief Wrapper de compatibilidade
     */
    void updateMapFromSensors(const cg_interfaces::msg::RobotSensors& sensors);
    
    // ========================================================================
    // LÓGICA DE DECISÃO (DFS + BACKTRACKING)
    // ========================================================================
    
    /**
     * @brief Decide próximo movimento usando DFS com backtracking
     * 
     * Prioridades:
     * 1. Se alvo adjacente → mover para ele
     * 2. Explorar célula não visitada adjacente (DFS)
     * 3. Backtracking usando pilha de caminho
     * 
     * @param visited Conjunto de células já visitadas
     * @param path_stack Pilha com caminho percorrido (para backtracking)
     * @return Direção do próximo movimento ou "" se impossível
     */
    std::string decideNextMoveWithBacktracking(
        const std::set<std::pair<int, int>>& visited,
        std::stack<std::pair<int, int>>& path_stack);
    
    /**
     * @brief Calcula direção para alcançar célula adjacente específica
     * @param target_row Linha de destino
     * @param target_col Coluna de destino
     * @return "up", "down", "left", "right" ou "" se não adjacente
     */
    std::string calculateDirectionToCell(int target_row, int target_col) const;
    
    /**
     * @brief Converte direção em offset de coordenadas
     * @param direction "up", "down", "left", "right"
     * @return Par (delta_row, delta_col)
     */
    std::pair<int, int> getDirectionOffset(const std::string& direction) const;
    
    // ========================================================================
    // BUSCA DE FRONTEIRAS (Para compatibilidade)
    // ========================================================================
    
    /**
     * @brief Encontra todas as fronteiras acessíveis
     * 
     * Fronteira = célula LIVRE adjacente a pelo menos uma célula UNKNOWN
     * 
     * @param visited Conjunto de células já visitadas
     * @return Vetor de posições (linha, coluna) das fronteiras
     */
    std::vector<std::pair<int, int>> findAccessibleFrontiers(
        const std::set<std::pair<int, int>>& visited) const;
    
    /**
     * @brief Encontra a fronteira mais próxima
     * @param frontiers Vetor de fronteiras disponíveis
     * @return Posição (linha, coluna) da fronteira mais próxima (Manhattan)
     */
    std::pair<int, int> findNearestFrontier(
        const std::vector<std::pair<int, int>>& frontiers) const;
    
    /**
     * @brief Wrapper - encontra célula não visitada mais próxima
     */
    std::pair<int, int> findNearestUnvisited(
        const std::set<std::pair<int, int>>& visited) const;
    
    /**
     * @brief Wrapper de compatibilidade
     */
    std::vector<std::pair<int, int>> findFrontiersForBFS(
        const std::set<std::pair<int, int>>& visited) const;
    
    // ========================================================================
    // NAVEGAÇÃO
    // ========================================================================
    
    /**
     * @brief Navega até uma célula específica usando A*
     * @param target_row Linha de destino
     * @param target_col Coluna de destino
     * @return true se conseguiu alcançar, false se caminho bloqueado
     */
    bool navigateToCell(int target_row, int target_col);
    
    /**
     * @brief Navega até o alvo (TARGET) usando A*
     * @return true se alcançou o alvo
     */
    bool navigateToTarget();
    
    /**
     * @brief Move o robô e atualiza o mapa
     * @param direction Direção: "up", "down", "left", "right"
     * @return true se movimento bem-sucedido
     */
    bool moveAndUpdate(const std::string& direction);
    
    /**
     * @brief Tenta movimento exploratório (fallback)
     * 
     * Usado quando nenhuma fronteira acessível é encontrada.
     * Tenta mover para célula adjacente não visitada.
     * 
     * @param visited Conjunto de células visitadas
     * @return true se conseguiu mover
     */
    bool tryExploratoryMove(const std::set<std::pair<int, int>>& visited);
    
    // ========================================================================
    // SINCRONIZAÇÃO E DETECÇÃO
    // ========================================================================
    
    /**
     * @brief Sincroniza posição do robô com servidor
     * 
     * Obtém mapa real do servidor e atualiza posição interna
     */
    void syncPositionWithServer();
    
    /**
     * @brief Verifica se o alvo foi detectado no mapa
     * @return true se existe célula TARGET no mapa
     */
    bool hasFoundTarget() const;
    
    /**
     * @brief Obtém posição do alvo no mapa
     * @return Par (linha, coluna) ou (-1, -1) se não encontrado
     */
    std::pair<int, int> getTargetPosition() const;
    
    // ========================================================================
    // UTILITÁRIOS
    // ========================================================================
    
    /**
     * @brief Exporta mapa para arquivo CSV
     * @param filename Nome do arquivo (será salvo em maps/)
     * @return true se exportação bem-sucedida
     */
    bool exportMapToCSV(const std::string& filename);
    
    /**
     * @brief Imprime mapa no console com emojis
     */
    void printMap() const;
    
    /**
     * @brief Define posição inicial do robô
     * @param row Linha inicial
     * @param col Coluna inicial
     */
    void setInitialPosition(int row, int col);
    
    /**
     * @brief Calcula porcentagem de cobertura do mapa
     * @return Percentual de células exploradas (não UNKNOWN)
     */
    float calculateCoverage() const;
    
    // ========================================================================
    // GETTERS
    // ========================================================================
    
    /**
     * @brief Obtém valor de uma célula do mapa
     * @return UNKNOWN, FREE, WALL, ROBOT ou TARGET
     */
    int getCellValue(int row, int col) const;
    
    /**
     * @brief Verifica se célula é navegável
     * @return true se FREE ou TARGET
     */
    bool isNavigable(int row, int col) const;
    
    /**
     * @brief Obtém vizinhos navegáveis de uma célula
     * @return Vetor de posições (linha, coluna)
     */
    std::vector<std::pair<int, int>> getNavigableNeighbors(int row, int col) const;
    
    /**
     * @brief Obtém posição atual do robô
     * @return Par (linha, coluna)
     */
    std::pair<int, int> getCurrentPosition() const { 
        return {current_row_, current_col_}; 
    }
    
    // ========================================================================
    // WRAPPERS DE COMPATIBILIDADE
    // ========================================================================
    
    void exploreDFS();
    void exploreDFS_Fixed();
    void exploreWallFollow();
    void exploreFrontier();
    
    std::vector<std::pair<int, int>> findAllFrontiers(
        const std::set<std::pair<int, int>>& visited) const;
    
    std::pair<int, int> findNearestUnexploredFrontier(
        const std::set<std::pair<int, int>>& visited) const;
    
    std::vector<std::pair<int, int>> findFrontierCells() const;
    
    bool isMappingComplete() const;
    
    std::string positionDiffToDirection(int row_diff, int col_diff) const;
    
    bool tryMove(const std::string& direction);
    
private:
    // ========================================================================
    // ATRIBUTOS PRIVADOS
    // ========================================================================
    
    ROSBridge& ros_bridge_;                        // Interface ROS2
    std::vector<std::vector<int>> internal_map_;   // Mapa ocupacional 2D
    
    int max_rows_;                                 // Dimensões do mapa
    int max_cols_;
    
    int current_row_;                              // Posição atual do robô
    int current_col_;
    std::string current_direction_;                // Direção atual
    
    // ========================================================================
    // MÉTODOS AUXILIARES PRIVADOS
    // ========================================================================
    
    /**
     * @brief Define valor de uma célula do mapa
     */
    void setCellValue(int row, int col, int value);
    
    /**
     * @brief Atualiza direção do robô
     */
    void updateDirection(const std::string& movement);
};

#endif // MAPPING_HPP