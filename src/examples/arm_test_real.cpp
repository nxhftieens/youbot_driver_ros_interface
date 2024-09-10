#include <iostream>
#include <assert.h>

#include "ros/ros.h"
#include "brics_actuator/JointPositions.h"

#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/io.hpp>

#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

using namespace std;

const double PI = 3.14159265358979323846;

int main(int argc, char **argv) {
	ros::init(argc, argv, "arm_test_real");
	ros::NodeHandle nh;
	ros::Publisher armPosPublisher;
    ros::Publisher gripperPosPublisher;

	armPosPublisher = nh.advertise<brics_actuator::JointPositions> ("arm_1/arm_controller/position_command", 1);
    gripperPosPublisher = nh.advertise<brics_actuator::JointPositions > ("arm_1/gripper_controller/position_command", 1);

	ros::Rate rate(50);
	double readValue;
	
	static const int numberOfArmJoints = 5;
	

	while(nh.ok()) {
		brics_actuator::JointPositions arm_command;
        vector <brics_actuator::JointValue> armJointPositions;
		armJointPositions.resize(numberOfArmJoints);

		brics_actuator::JointPositions gripper_command;
        vector <brics_actuator::JointValue> gripperJointPositions;
		gripperJointPositions.resize(2);

        stringstream jointName;

		for (int i = 0; i < numberOfArmJoints; i++) {
			cout << "Please type in value for joint " << i + 1 << endl;
			cin >> readValue;
			jointName.str("");
			jointName << "arm_joint_" << (i + 1);
			armJointPositions[i].joint_uri = jointName.str();
			armJointPositions[i].value = readValue * PI / 180;
			armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);
		}

		armJointPositions[0].value = - armJointPositions[0].value + 169 * PI / 180; //from -169 to 169
        armJointPositions[1].value = 155 * PI / 180 + armJointPositions[1].value; //from -6 to -154
        armJointPositions[2].value = armJointPositions[2].value - 146 * PI / 180; //from -141 to 145
        armJointPositions[3].value = armJointPositions[3].value + 102.5 * PI / 180; //from -93 to 93 (can be reduce to -90 to 90 easier implementation)
        armJointPositions[4].value = - armJointPositions[4].value + 167.5 * PI / 180; //from -155 to 160

        cout << "Please type in value for gripper's left finger (from 0.0 to 0.0115):" << endl;
        cin >> readValue;
        gripperJointPositions[0].joint_uri = "gripper_finger_joint_l";
		gripperJointPositions[0].value = readValue;
		gripperJointPositions[0].unit = boost::units::to_string(boost::units::si::meter);

        cout << "Please type in value for gripper's right finger (from 0.0 to 0.0115):" << endl;
        cin >> readValue;
        gripperJointPositions[1].joint_uri = "gripper_finger_joint_r";
		gripperJointPositions[1].value = readValue;
		gripperJointPositions[1].unit = boost::units::to_string(boost::units::si::meter);

		cout << "sending command ..." << endl;
        arm_command.positions = armJointPositions;
        gripper_command.positions = gripperJointPositions;
		armPosPublisher.publish(arm_command);
		gripperPosPublisher.publish(gripper_command);
		cout << "--------------------" << endl;
		ros::spinOnce();
		rate.sleep();
	}

	return 0;

}