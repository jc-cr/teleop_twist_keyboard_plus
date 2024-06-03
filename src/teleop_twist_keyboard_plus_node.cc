#include "teleop_twist_keyboard_plus.hh"
#include <signal.h>

void quit(int sig) {
    ros::shutdown();
    exit(0);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "teleop_twist_keyboard_plus");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    signal(SIGINT, quit);

    TeleopTwistKeyboardPlus teleop_twist_keyboard_plus(nh, pnh);
    teleop_twist_keyboard_plus.keyLoop();

    return 0;
}
