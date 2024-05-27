#include "teleop_twist_keyboard_plus.hh"

#include <ros/package.h>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <signal.h>

// Signal handler for graceful shutdown
void quit(int sig) {
    ros::shutdown();
    exit(0);
}

TeleopTwistKeyboardPlus::TeleopTwistKeyboardPlus(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    : _nh(nh), _speed(0.5), _turn(1.0), _speed_limit(1000), _turn_limit(1000), _key_timeout(0.5)
{
    try
    {
        pnh.param("speed", _speed, _speed);
        pnh.param("turn", _turn, _turn);
        pnh.param("speed_limit", _speed_limit, _speed_limit);
        pnh.param("turn_limit", _turn_limit, _turn_limit);
        pnh.param("key_timeout", _key_timeout, _key_timeout);

        _cmd_vel_pub = _nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

        // Construct the full path using the package path
        std::string package_path = ros::package::getPath("teleop_twist_keyboard_plus");
        std::string config_path = package_path + "/" + "config/bindings.yaml";

        pnh.param("config_file", config_path, config_path);
        loadBindings(config_path);
    }
    catch (const ros::Exception &e)
    {
        ROS_ERROR("Failed to initialize teleop_twist_keyboard_plus: %s", e.what());
        ros::shutdown();
        exit(1);
    }
}

TeleopTwistKeyboardPlus::~TeleopTwistKeyboardPlus()
{
}

void TeleopTwistKeyboardPlus::loadBindings(const std::string &config_file)
{
    try
    {
        YAML::Node config = YAML::LoadFile(config_file);
        for (const auto &item : config["move_bindings"])
        {
            char key = item.first.as<char>();
            std::vector<float> values = item.second.as<std::vector<float>>();
            _moveBindings[key] = values;
        }
        for (const auto &item : config["speed_bindings"])
        {
            char key = item.first.as<char>();
            std::vector<float> values = item.second.as<std::vector<float>>();
            _speedBindings[key] = values;
        }
    }
    catch (const YAML::Exception &e)
    {
        ROS_ERROR("Failed to load bindings from %s: %s", config_file.c_str(), e.what());
        ros::shutdown();
        exit(1);
    }
}

void TeleopTwistKeyboardPlus::keyLoop()
{
    char c;
    int kfd = 0;
    struct termios cooked, raw;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("Use 'q'/'z' to increase/decrease max speeds by 10%");
    puts("Use 'w'/'x' to increase/decrease only linear speed by 10%");
    puts("Use 'e'/'c' to increase/decrease only angular speed by 10%");
    puts("Press Ctrl-C to quit");

    while (ros::ok())
    {
        // get the next event from the keyboard
        if (read(kfd, &c, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }

        if (_moveBindings.find(c) != _moveBindings.end())
        {
            _twist_msg.linear.x = _moveBindings[c][0] * _speed;
            _twist_msg.linear.y = _moveBindings[c][1] * _speed;
            _twist_msg.linear.z = _moveBindings[c][2] * _speed;
            _twist_msg.angular.x = 0;
            _twist_msg.angular.y = 0;
            _twist_msg.angular.z = _moveBindings[c][3] * _turn;
        }
        else if (_speedBindings.find(c) != _speedBindings.end())
        {
            _speed = std::min(_speed_limit, _speed * _speedBindings[c][0]);
            _turn = std::min(_turn_limit, _turn * _speedBindings[c][1]);
            printf("currently:\tspeed %f\tturn %f\n", _speed, _turn);
        }
        else
        {
            // Skip updating cmd_vel if key timeout and robot already stopped
            if (c == 0 && _twist_msg.linear.x == 0 && _twist_msg.linear.y == 0 &&
                _twist_msg.linear.z == 0 && _twist_msg.angular.z == 0)
            {
                continue;
            }
            _twist_msg.linear.x = 0;
            _twist_msg.linear.y = 0;
            _twist_msg.linear.z = 0;
            _twist_msg.angular.x = 0;
            _twist_msg.angular.y = 0;
            _twist_msg.angular.z = 0;
            if (c == '\x03')
            {
                break;
            }
        }

        _cmd_vel_pub.publish(_twist_msg);
    }

    tcsetattr(kfd, TCSANOW, &cooked);
}
