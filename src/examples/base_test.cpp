#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>

int main(int argc, char ** argv) {
    ros::init(argc, argv, "base_test");
    ros::NodeHandle nh;

    ros::Publisher pub;
    pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    double vx = 0.0;
    double vy = 0.0;
    double th = 0.0;

    ros::Rate rate(10);
    
    while (ros::ok()) {
        std::cout << "Please type value of vx" << std::endl;
        std::cin >> vx;
        std::cout << "Please type value of vy" << std::endl;
        std::cin >> vy;
        std::cout << "Please type value of th" << std::endl;
        std::cin >> th;
        
        geometry_msgs::Twist msg;
        msg.linear.x = vx;
        msg.linear.y = vy;
        msg.angular.z = th;

        pub.publish(msg);
        rate.sleep();
    }
}
