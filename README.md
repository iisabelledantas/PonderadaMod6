# Desafio de Navegação em Labirinto

Sistema de navegação e mapeamento autônomo desenvolvido em C++ com ROS2 para resolver desafios de exploração de labirintos.

## Sobre o Projeto

Este projeto implementa um sistema completo de navegação autônoma para robôs em ambientes de labirinto, dividido em duas partes principais:

### Parte 1: Navegação com Mapa Conhecido
- Utiliza o algoritmo de **Dijkstra** para encontrar o caminho otimizado
- Recebe o mapa completo do ambiente
- Planeja e executa a rota mais eficiente até o alvo

### Parte 2: Exploração e Mapeamento SLAM
- Implementa **DFS (Depth-First Search) com Backtracking**
- Constrói um mapa ocupacional do ambiente desconhecido
- Utiliza sensores para detectar obstáculos e o alvo
- Navega autonomamente até alcançar o objetivo

### Componentes

1. **ROSBridge** (`ros_bridge.cpp/hpp`)
   - Interface com o sistema ROS2
   - Comunicação com serviços (`/get_map`, `/move_command`, `/reset`)
   - Subscrição de sensores (`/culling_games/robot_sensors`)

2. **Mapper** (`mapping.cpp/hpp`)
   - Implementação do algoritmo SLAM
   - Exploração DFS com backtracking
   - Atualização do mapa ocupacional
   - Detecção e navegação até o alvo

3. **Navigation** (`navigation.cpp/hpp`)
   - Implementação do algoritmo de Dijkstra
   - Planejamento de rotas otimizadas

4. **Graph** (`graph.cpp/hpp`)
   - Representação do ambiente como grafo
   - Validação de células navegáveis
   - Geração de vizinhos e reconstrução de caminhos

## Estrutura do Código

### Arquivos Principais

```
modulo6/
├── include/
│   ├── graph.hpp          
│   ├── mapping.hpp        
│   ├── navigation.hpp     
│   └── ros_bridge.hpp     
├── src/
│   ├── main.cpp           
│   ├── graph.cpp          
│   ├── mapping.cpp        
│   ├── navigation.cpp     
│   └── ros_bridge.cpp    
```

## Execução

1. Compile o pacote:
```bash
colcon build 
```

2. Source o workspace:
```bash
source install/setup.bash
```

3. Inicie o simulador do labirinto:
```bash
ros2 run cg maze
```

4. Em outro terminal, execute o programa principal:
```bash
ros2 run modulo6 maze_solver
```
