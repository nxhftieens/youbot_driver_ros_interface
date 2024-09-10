#include <iostream>
#include <assert.h>

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

using namespace std;

const double PI = 3.14159265358979323846;

int main(int argc, char **argv) {
	ros::init(argc, argv, "arm_test");
	ros::NodeHandle nh;
	ros::Publisher armPosPublisher;
    ros::Publisher gripperPosPublisher;

	armPosPublisher = nh.advertise<std_msgs::Float64MultiArray>("arm_1/arm_controller/command", 1);
    gripperPosPublisher = nh.advertise<std_msgs::Float64MultiArray>("arm_1/gripper_controller/command", 1);

	ros::Rate rate(50);
	float readValue;
	
	static const int numberOfArmJoints = 5;
	

	while(nh.ok()) {
		std_msgs::Float64MultiArray arm_command;
		arm_command.data.resize(numberOfArmJoints);
		std_msgs::Float64MultiArray gripper_command;
		gripper_command.data.resize(2);

		double theta[5] = {0};

		for (int i = 0; i < numberOfArmJoints; i++) {
			cout << "Please type in value (deg) for joint " << i + 1 << endl;
			cin >> readValue;
			theta[i] = readValue;
		}

		theta[0] = - theta[0] + 170; // from -170 to 170
        theta[1] = 155 + theta[1]; // from 0 to -155
        theta[2] = theta[2] - 146; // from -146 to 151
        theta[3] = theta[3] + 102.5; // from -102.5 to 102.5
        theta[4] = - theta[4] + 167.5; // from -167.5 to 167.5

		for (int i = 0; i < numberOfArmJoints; i++) {
			arm_command.data[i] = theta[i] * PI / 180;
			cout << "Joint " << i + 1 << " = " << arm_command.data[i] << " rad" << endl;
		}

        cout << "Please type in value for gripper's left finger (from 0.0 to 0.0125):" << endl;
        cin >> readValue;
        gripper_command.data[0] = readValue;
        cout << "Please type in value for gripper's right finger (from 0.0 to 0.0125):" << endl;
        cin >> readValue;
        gripper_command.data[1] = readValue;

		cout << "sending command ..." << endl;
		armPosPublisher.publish(arm_command);
		gripperPosPublisher.publish(gripper_command);
		cout << "--------------------" << endl;
		ros::spinOnce();
		rate.sleep();
	}

	return 0;

}