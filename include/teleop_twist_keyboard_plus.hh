#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <yaml-cpp/yaml.h>
#include <map>
#include <unordered_map>
#include <string>
#include <functional>
#include <std_msgs/String.h>

class TeleopTwistKeyboardPlus
{
public:
    TeleopTwistKeyboardPlus(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~TeleopTwistKeyboardPlus();
    void keyLoop();

private:
    void _loadBindings(const std::string &config_file);
    void _printHelpMessage();

    ros::NodeHandle _nh;
    ros::Publisher _cmd_vel_pub;

    std::unordered_map<char, std::vector<float>> _moveBindings;
    std::unordered_map<char, std::string> _actionBindings; 
    std::unordered_map<char, std::vector<float>> _holonomicMoveBindings;
    std::unordered_map<char, std::string> _holonomicActionBindings; 
    std::unordered_map<char, std::pair<float, float>> _speedBindings;
    std::unordered_map<char, std::function<void()>> _customBindings;


    std::vector<std::string> _custom_item_description;

    float _speed;
    float _turn;
    float _speed_limit;
    float _turn_limit;
    float _key_timeout;

    geometry_msgs::Twist _twist_msg;
};
