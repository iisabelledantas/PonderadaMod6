cmake_minimum_required(VERSION 3.8)
project(modulo6)

# Configurações de compilador C++17
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Encontrar dependências do ROS2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cg_interfaces REQUIRED)

# Incluir diretórios de headers
include_directories(include)

# Adicionar executável principal com todos os arquivos fonte
add_executable(maze_solver
  src/main.cpp
  src/ros_bridge.cpp
  src/graph.cpp
  src/navigation.cpp
  src/mapping.cpp
)

# Linkar dependências do ROS2
ament_target_dependencies(maze_solver
  rclcpp
  std_msgs
  cg_interfaces
)

# Instalar executável
install(TARGETS
  maze_solver
  DESTINATION lib/${PROJECT_NAME}
)

# Instalar diretórios adicionais (opcional)
install(DIRECTORY
  maps/
  DESTINATION share/${PROJECT_NAME}/maps
)

ament_package()