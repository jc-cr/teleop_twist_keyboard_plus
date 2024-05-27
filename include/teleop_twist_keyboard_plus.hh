#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class TeleopTwistKeyboardPlus
{
public:
    TeleopTwistKeyboardPlus(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~TeleopTwistKeyboardPlus();
    void keyLoop();

private:
    void loadBindings(const std::string &config_file);

    ros::NodeHandle _nh;
    ros::Publisher _cmd_vel_pub;

    std::map<char, std::vector<float>> _moveBindings;
    std::map<char, std::vector<float>> _speedBindings;

    float _speed;
    float _turn;
    float _speed_limit;
    float _turn_limit;
    float _key_timeout;

    geometry_msgs::Twist _twist_msg;
};
