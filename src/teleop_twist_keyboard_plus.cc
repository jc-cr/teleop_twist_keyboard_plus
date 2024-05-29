#include "teleop_twist_keyboard_plus.hh"
#include <ros/package.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <signal.h>
#include <std_msgs/String.h>

static struct termios cooked;
static int kfd = 0;

// Function to reset the terminal settings
void static resetTerminal()
{
    tcsetattr(kfd, TCSANOW, &cooked);
}

// Signal handler for graceful shutdown
void static signalHandler(int sig)
{
    resetTerminal();
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
        _loadBindings(config_path);
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

void TeleopTwistKeyboardPlus::_loadBindings(const std::string &config_file)
{
    try
    {
        YAML::Node config = YAML::LoadFile(config_file);

        for (const auto &item : config["move_bindings"])
        {
            char key = item.second.as<std::string>()[0];
            std::string action = item.first.as<std::string>();

            // Note: The particualr order here is important as we use it to print the help message
            if (action == "forward_left")
                {
                    _moveBindings[key] = {1, 0, 0, 1}; // Forward-Left
                }
            else if (action == "forward")
                {
                    _moveBindings[key] = {1, 0, 0, 0}; // Forward
                }
            else if (action == "forward_right")
                {
                    _moveBindings[key] = {1, 0, 0, -1}; // Forward-Right
                }
            else if (action == "left")
                {
                    _moveBindings[key] = {0, 0, 0, 1}; // Left
                }
            else if (action == "no_movement")
                {
                }
            else if (action == "right")
                {
                    _moveBindings[key] = {0, 0, 0, -1}; // Right
                }
            else if (action == "backward_left")
                {
                    _moveBindings[key] = {-1, 0, 0, 1}; // Backward-Left
                }
            else if (action == "backward")
                {
                    _moveBindings[key] = {-1, 0, 0, 0}; // Backward
                }
            else if (action == "backward_right")
                {
                    _moveBindings[key] = {-1, 0, 0, -1}; // Backward-Right
                }
            else if (action == "up")
                {
                    _moveBindings[key] = {0, 0, 1, 0}; // Up
                }
            else if (action == "down")
                {
                    _moveBindings[key] = {0, 0, -1, 0}; // Down
                }
        }

        for (const auto &item : config["holonomic_move_bindings"])
        {
            char key = item.second.as<std::string>()[0];
            std::string action = item.first.as<std::string>();

            if (action == "holonomic_forward")
            {
                _holonomicMoveBindings[key] = {1, 0, 0, 0}; // Holonomic Forward
            }
            else if (action == "holonomic_backward")
            {
                _holonomicMoveBindings[key] = {-1, 0, 0, 0}; // Holonomic Backward
            }
            else if (action == "holonomic_left")
            {
                _holonomicMoveBindings[key] = {0, 1, 0, 0}; // Holonomic Left
            }
            else if (action == "holonomic_no_movement")
            {

            }
            else if (action == "holonomic_right")
            {
                _holonomicMoveBindings[key] = {0, -1, 0, 0}; // Holonomic Right
            }
            else if (action == "holonomic_forward_left")
            {
                _holonomicMoveBindings[key] = {1, 1, 0, 0}; // Holonomic Forward-Left
            }
            else if (action == "holonomic_forward_right")
            {
                _holonomicMoveBindings[key] = {1, -1, 0, 0}; // Holonomic Forward-Right
            }
            else if (action == "holonomic_backward_left")
            {
                _holonomicMoveBindings[key] = {-1, 1, 0, 0}; // Holonomic Backward-Left
            }
            else if (action == "holonomic_backward_right")
            {
                _holonomicMoveBindings[key] = {-1, -1, 0, 0}; // Holonomic Backward-Right
            }
        }

        for (const auto &item : config["speed_bindings"])
        {
            char key = item.second.as<std::string>()[0];
            std::string action = item.first.as<std::string>();

            if (action == "increase_max_speed_by_10")
            {
                _speedBindings[key] = {1.1, 1.1};
            }
            else if (action == "decrease_max_speed_by_10")
            {
                _speedBindings[key] = {0.9, 0.9};
            }
            else if (action == "decrease_linear_speed_by_10")
            {
                _speedBindings[key] = {0.9, 1.0};
            }
            else if (action == "increase_linear_speed_by_10")
            {
                _speedBindings[key] = {1.1, 1.0};
            }
            else if (action == "decrease_angular_speed_by_10")
            {
                _speedBindings[key] = {1.0, 0.9};
            }
            else if (action == "increase_angular_speed_by_10")
            {
                _speedBindings[key] = {1.0, 1.1};
            }
        }

        for (const auto &item : config["custom_bindings"]["publish"])
        {
            char key = item.second.as<std::string>()[0];
            std::string topic = item.first.as<std::string>();
            _customBindings[key] = [this, topic]()
            {
                std_msgs::String msg;
                msg.data = "trigger";
                ros::Publisher pub = _nh.advertise<std_msgs::String>(topic, 1);
                pub.publish(msg);
            };
        }
    }
    catch (const YAML::Exception &e)
    {
        ROS_ERROR("Failed to load bindings from %s: %s", config_file.c_str(), e.what());
        ros::shutdown();
        exit(1);
    }
}

// Reading from the keyboard  and Publishing to Twist!
// ---------------------------
// Moving around:
   // u    i    o
   // j    k    l
   // m    ,    .
// 
// For Holonomic mode (strafing), hold down the shift key:
// ---------------------------
   // U    I    O
   // J    K    L
   // M    <    >
// 
// t : up (+z)
// b : down (-z)
// 
// anything else : stop
// 
// q/z : increase/decrease max speeds by 10%
// w/x : increase/decrease only linear speed by 10%
// e/c : increase/decrease only angular speed by 10%
// 
// CTRL-C to quit
// """

void TeleopTwistKeyboardPlus::_printHelpMessage()
{
    std::cout << "Reading from the keyboard and publishing to Twist!\n";
    std::cout << "---------------------------\n";
    std::cout << "Moving around:\n";

    auto printRow = [](const std::map<char, std::vector<float>>& bindings, const std::initializer_list<std::string>& order) {
        for (const auto& action : order)
        {
            char key = action[0];
            if (bindings.count(key))
            {
                std::cout << key << "    ";
            }
            else
            {
                std::cout << "     ";
            }
        }
        std::cout << "\n";
    };

    // Print the standard movement keys in a 3x3 grid
    printRow(_moveBindings, {"u", "i", "o"});
    printRow(_moveBindings, {"j", "k", "l"});
    printRow(_moveBindings, {"m", ",", "."});

    std::cout << "\nFor Holonomic mode (strafing), hold down the shift key:\n";
    std::cout << "---------------------------\n";

    // Print the holonomic movement keys in a 3x3 grid
    printRow(_holonomicMoveBindings, {"U", "I", "O"});
    printRow(_holonomicMoveBindings, {"J", "K", "L"});
    printRow(_holonomicMoveBindings, {"M", "<", ">"});

    std::cout << "\n";

    // Print the vertical movement keys
    std::cout << "t : up (+z)\n";
    std::cout << "b : down (-z)\n\n";

    std::cout << "anything else : stop\n\n";

    // Print the speed adjustment keys
    std::cout << "q/z : increase/decrease max speeds by 10%\n";
    std::cout << "w/x : increase/decrease only linear speed by 10%\n";
    std::cout << "e/c : increase/decrease only angular speed by 10%\n\n";

    std::cout << "CTRL-C to quit" << std::endl;
}



void TeleopTwistKeyboardPlus::keyLoop()
{
    char c;
    kfd = 0;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    struct termios raw;
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    // Register signal handler for CTRL-C
    signal(SIGINT, signalHandler);

    _printHelpMessage();

    while (ros::ok())
    {
        // get the next event from the keyboard
        if (read(kfd, &c, 1) < 0)
        {
            perror("read():");
            resetTerminal();
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
        else if (_holonomicMoveBindings.find(c) != _holonomicMoveBindings.end())
        {
            _twist_msg.linear.x = _holonomicMoveBindings[c][0] * _speed;
            _twist_msg.linear.y = _holonomicMoveBindings[c][1] * _speed;
            _twist_msg.linear.z = _holonomicMoveBindings[c][2] * _speed;
            _twist_msg.angular.x = 0;
            _twist_msg.angular.y = 0;
            _twist_msg.angular.z = _holonomicMoveBindings[c][3] * _turn;
        }
        else if (_speedBindings.find(c) != _speedBindings.end())
        {
            _speed = std::min(_speed_limit, _speed * _speedBindings[c].first);
            _turn = std::min(_turn_limit, _turn * _speedBindings[c].second);
            printf("currently:\tspeed %f\tturn %f\n", _speed, _turn);
        }
        else if (_customBindings.find(c) != _customBindings.end())
        {
            _customBindings[c]();
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

    resetTerminal();
}