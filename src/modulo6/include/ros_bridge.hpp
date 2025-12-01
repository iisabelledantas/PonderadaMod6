#ifndef ROS_BRIDGE_HPP
#define ROS_BRIDGE_HPP
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>
#include "cg_interfaces/srv/get_map.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"
#include "cg_interfaces/srv/reset.hpp"
#include "cg_interfaces/msg/robot_sensors.hpp"

class ROSBridge {
public:

    explicit ROSBridge(rclcpp::Node::SharedPtr node);

    std::vector<std::vector<int>> getMap();

    bool moveRobot(const std::string& direction);

    bool resetEnvironment();

    cg_interfaces::msg::RobotSensors getSensorData();

    std::pair<int, int> findRobotPosition(const std::vector<std::vector<int>>& map);

    std::pair<int, int> findTargetPosition(const std::vector<std::vector<int>>& map);

private:
    rclcpp::Node::SharedPtr node_;  
    
    rclcpp::Client<cg_interfaces::srv::GetMap>::SharedPtr get_map_client_;
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_cmd_client_;
    rclcpp::Client<cg_interfaces::srv::Reset>::SharedPtr reset_client_;
    
    rclcpp::Subscription<cg_interfaces::msg::RobotSensors>::SharedPtr sensor_sub_;
    
    cg_interfaces::msg::RobotSensors last_sensor_data_;
    
    void sensorCallback(const cg_interfaces::msg::RobotSensors::SharedPtr msg);
};

#endif 