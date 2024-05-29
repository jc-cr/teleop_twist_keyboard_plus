#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <yaml-cpp/yaml.h>
#include <map>
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

    std::map<char, std::vector<float>> _moveBindings;
    std::map<char, std::string> _actionBindings; 
    std::map<char, std::vector<float>> _holonomicMoveBindings;
    std::map<char, std::string> _holonomicActionBindings; 
    std::map<char, std::pair<float, float>> _speedBindings;
    std::map<char, std::function<void()>> _customBindings;

    float _speed;
    float _turn;
    float _speed_limit;
    float _turn_limit;
    float _key_timeout;

    geometry_msgs::Twist _twist_msg;
};
