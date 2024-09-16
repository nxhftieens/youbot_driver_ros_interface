#include <iostream>
#include <cmath>
#include <array>

#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "brics_actuator/CartesianWrench.h"
#include <geometry_msgs/Twist.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "brics_actuator/JointPositions.h"

#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/io.hpp>

#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

using namespace std;

const double PI = 3.14159265358979323846;
const double ALMOST_00 = 0.001;
const double ALMOST_0 = 0.01;
const double eps = 0.000001;

const double a1 = 0.033;
const double alpha1 = -PI / 2;
const double d1 = 0.147;

const double a2 = 0.155;
const double alpha2 = 0;
const double d2 = 0;

const double a3 = 0.135;
const double alpha3 = 0;
const double d3 = 0;

const double a4 = 0;
const double alpha4 = -PI / 2;
const double d4 = 0;

const double a5 = 0;
const double alpha5 = 0;
const double d5 = 0.2175;

const int TOTAL_ARM_JOINTS = 5;

double radToDeg(double rad) {
    return rad * 180 / PI;
}

double degToRad(double deg) {
    return deg * PI / 180;
}

int sign(double num) {
    if (num >= 0) return 1;
    else return -1;
}

bool approximatelyEqual(double a, double b, double epsilon)
{
    // return fabs(a - b) <= ( (fabs(a) < fabs(b) ? fabs(b) : fabs(a)) * epsilon);
    return fabs(a - b) <= epsilon;
}

void inputTarget(double& roll, double& pitch, double& target_z, double& target_p, double& target_x, double& target_y, double base_height, int& p_type) {
    bool z_OK = false;
    bool p_OK = false;
    double readValue;
    while (!z_OK || !p_OK) {
        z_OK = false;
        p_OK = false;

        //Input pitch angle
        cout << "Please type in pitch angle required (degree): ";
        cin >> readValue;
        pitch = readValue;

        //Input target_z
        //Theta 4 limit from -90 to 90;
        double z_wrist_max;
        if (pitch >= 0 && pitch <= 90) {
            z_wrist_max = a2 + a3 * cos(degToRad(pitch));
        } else if (pitch > 90 && pitch <= 180) {
            z_wrist_max = a2 - a3 * cos(degToRad(pitch));
        } else {
            z_wrist_max = a2 + a3;
        }
        double z_max = base_height + z_wrist_max + d1 - d5 * sin (degToRad(pitch));
        double z_min = 0;
        cout << "Please type in z target between " << z_min << " and " << z_max << " : ";
        cin >> readValue;
        target_z = readValue;
        if (target_z <= z_max && target_z >= z_min) {
            z_OK = true;
        } else {
            cout << "Out of range!!!" << endl;
            continue;
        }

        //Input target_p
        double a2_lowest_I, a2_lowest_II;
        double p_min_I = 0;
        double p_min_II = 0;
        double z_wrist = target_z - base_height - d1 + d5 * sin(degToRad(pitch));
        if (pitch >= 0 && pitch <= 180) {
            a2_lowest_I = z_wrist - a3 * cos(degToRad(pitch));
            a2_lowest_II = z_wrist + a3 * cos(degToRad(pitch));
            p_min_I += a3 * sin(degToRad(pitch));
            p_min_II += -a3 * sin(degToRad(pitch));
        } else {
            a2_lowest_I = z_wrist - a3;
            a2_lowest_II = z_wrist - a3;
        }
        bool p_I, p_II;
        if (a2_lowest_I - ALMOST_00 <= a2) {
            p_I = true;
        } else {
            p_I = false;
        }
        if (a2_lowest_II - ALMOST_00 <= a2) {
            p_II = true;
        } else {
            p_II = false;
        }        

        double s_phi = z_wrist / (a2 + a3);
        double c_phi_I = sqrt(sqrt(pow(1 - pow(s_phi, 2), 2)));
        double c_phi_II = - sqrt(sqrt(pow(1 - pow(s_phi, 2), 2)));
        double phi_I = atan2(s_phi, c_phi_I);
        double phi_II = atan2(s_phi, c_phi_II);

        double p_max_I, p_max_II;
        if (p_I) {
            if (radToDeg(phi_I) <= (90 - pitch)) {
                p_max_I = sqrt(sqrt(pow(pow(a2 + a3, 2) - pow(z_wrist, 2), 2))); 
            } else {
                p_max_I = a3 * cos(degToRad(90 - pitch)) + sqrt(sqrt(pow(pow(a2, 2) - pow(a2_lowest_I, 2), 2)));
            }
            if (a2_lowest_I >= (a2 * sin(degToRad(154)))) {
                p_min_I += -sqrt(sqrt(pow(pow(a2, 2) - pow(a2_lowest_I, 2), 2)));
            } else {
                p_min_I += a2 * cos(degToRad(154));
            }
        }

        if (p_II) {
            if (radToDeg(phi_II) >= (270 - pitch)) {
                if (radToDeg(phi_II) >= 154) {
                    p_max_II = -sqrt(sqrt(pow(pow(a3, 2) - pow(z_wrist - a2 * sin(degToRad(154)), 2), 2))) + a2 * cos(degToRad(154));
                } else {
                    p_max_II = -sqrt(sqrt(pow(pow(a2 + a3, 2) - pow(z_wrist, 2), 2)));
                }
            } else {
                if (a2_lowest_II >= (a2 * sin(degToRad(154)))) {
                    p_max_II = -a3 * cos(degToRad(pitch - 90)) - sqrt(sqrt(pow(pow(a2, 2) - pow(a2_lowest_II, 2), 2)));
                } else {
                    p_max_II = -a3 * cos(degToRad(pitch - 90)) - sqrt(sqrt(pow(pow(a2, 2) - pow(a2 * sin(degToRad(154)), 2), 2)));
                }
                
            }
            p_min_II += sqrt(sqrt(pow(pow(a2, 2) - pow(a2_lowest_II, 2), 2)));
        }

        //Safety offset
        if (pitch > 0 && pitch < 180) {
            if (p_min_I < a3 - a1) {
                p_min_I = a3 - a1;
            }
            if (p_min_II > -a3 - a1) {
                p_min_II = -a3 - a1;
            } 
        }       

        if (p_min_I > p_max_I) {
            p_I = false;
        }
        if (p_min_II < p_max_II) {
            p_II = false;
        }

        if (!p_I && !p_II) {
            p_OK = false;
            continue;
        }

        cout << "Please type in p2 target:" << endl;;
        if (p_I) {
            cout << "\tbetween " << p_max_I << " and " << p_min_I << endl;
        }
        if (p_II) {
            cout << "\tbetween " << p_min_II << " and " << p_max_II << endl;
        }
        cin >> readValue;
        target_p = readValue;
        if ((target_p <= p_max_I && target_p >= p_min_I) || (target_p <= p_min_II && target_p >= p_max_II)) {
            p_OK = true;
        } else {
            cout << "Out of range!!!" << endl;
            continue;
        }

        if (target_p <= p_max_I && target_p >= p_min_I) {
            p_type = 1;
        } else {
            p_type = -1;
        }

        //Input roll angle
        cout << "Please type in roll angle required (degree): ";
        cin >> readValue;
        roll = readValue;
        //Input x target
        cout << "Please type in x target: ";
        cin >> readValue;
        target_x = readValue;
        //Input y target
        cout << "Please type in y target: ";
        cin >> readValue;
        target_y = readValue;
        
    }
    cout << "===========================================" << endl << endl;
    cout << "Pitch: " << pitch << " degrees = " << degToRad(pitch) << " rad" << endl;
    cout << "Roll: " << roll << " degrees = " << degToRad(roll) << " rad" << endl;
    cout << "z_target: " << target_z << " p_target: " << target_p << endl;
    cout << "x_target: " << target_x << " y_target: " << target_y << endl;    
    cout << "===========================================" << endl << endl;
}

/*
Inverse kinematics solved by using geometry. Origin (for z-wrist and p) is joint 2. CW is positive
Input:  x, y - from target to base (arm link 0)
        zw, p - from joint 2
*/
void inverseKinematics(double zw, double p, double roll, double pitch,
                        double& theta2, double& theta3, double& theta4, double& theta5, bool opt = true) {
    double c_theta3 = -(pow(a2, 2) + pow(a3, 2) - pow(p, 2) - pow(zw, 2)) / (2 * a2 * a3);
    double s_theta3_I = sqrt(sqrt(pow(1 - pow(c_theta3, 2), 2)));
    double s_theta3_II = -sqrt(sqrt(pow(1 - pow(c_theta3, 2), 2)));
    double theta3_I = atan2(s_theta3_I, c_theta3);
    double theta3_II = atan2(s_theta3_II, c_theta3);

    double phi1 = atan2(zw, p);
    double k1 = a2 + a3 * c_theta3;
    double k2_I = a3 * s_theta3_I;
    double k2_II = a3 * s_theta3_II;
    double phi2_I = atan2(k2_I, k1);
    double phi2_II = atan2(k2_II, k1);
    double theta2_I = -(phi1 + phi2_I);
    double theta2_II = -(phi1 + phi2_II);

    double theta4_I = degToRad(pitch) - theta2_I - theta3_I;
    double theta4_II = degToRad(pitch) - theta2_II - theta3_II;

    theta5 = degToRad(roll);

    if (opt == true) {
        theta2 = theta2_I;
        theta3 = theta3_I;
        theta4 = theta4_I;
    }
    else {
        theta2 = theta2_II;
        theta3 = theta3_II;
        theta4 = theta4_II;
    }
    if (theta4 > PI) {
        theta4 -= 2 * PI;
    } else if (theta4 < -PI) {
        theta4 += 2 * PI;
    }    
}

void baseAdjustment(double& base_x, double& base_y, double target_x, double target_y, double p, double pitch, int p_type, double& joint1, tf2_ros::Buffer* tfBuffer, ros::Publisher* baseVelPublisher) {
    ros::Rate rate(50);
    geometry_msgs::Twist vel_msg;
    geometry_msgs::TransformStamped transformStamped;
    joint1 = atan2(target_y - base_y, target_x - base_x);
    // double angle_to_target = atan2(target_y - base_y, target_x - base_x);
    bool out_of_reach = true;

    double kp_y, ki_y, kd_y, dt, integral_y, prev_error_y;
    double kp_x, ki_x, kd_x, integral_x, prev_error_x;
    dt = 0.02;
    integral_y = 0.0;
    prev_error_y = 0.0;
    integral_x = 0.0;
    prev_error_x = 0.0;
    kp_y = 0.15;
    ki_y = 0.05;
    kd_y = 0.01;
    kp_x = 0.15;
    ki_x = 0.05;
    kd_x = 0.01;
    do {
        try {
            transformStamped = (*tfBuffer).lookupTransform("odom", "arm_link_0", ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        base_x = transformStamped.transform.translation.x - 0.143;
        base_y = transformStamped.transform.translation.y;

        double error_y = -(target_y - (p + a1 + d5 * cos(degToRad(pitch))) * sin(joint1)) - base_y;
        double P_y = error_y * kp_y;
        integral_y += error_y * dt;
        double I_y = integral_y * ki_y;
        double D_y = kd_y * (error_y - prev_error_y) / dt;        
        if ((P_y + I_y + D_y) > 0.1) {
            vel_msg.linear.y = 0.1;
            integral_y += (0.1 - (P_y + I_y + D_y)) * 0.1;
        } else if ((P_y + I_y + D_y) < -0.1) {
            vel_msg.linear.y = -0.1;
            integral_y += (-0.1 - (P_y + I_y + D_y)) * 0.1;
        } else {
            vel_msg.linear.y = (P_y + I_y + D_y);
        }
        prev_error_y = error_y;

        double error_x = -(target_x - (p + a1 + d5 * cos(degToRad(pitch))) * cos(joint1)) - base_x;
        double P_x = error_x * kp_x;
        integral_x += error_x * dt;
        double I_x = integral_x * ki_x;
        double D_x = kd_x * (error_x - prev_error_x) / dt;
        if ((P_x + I_x + D_x) > 0.1) {
            vel_msg.linear.x = 0.1;
            integral_x += (0.1 - (P_x + I_x + D_x)) * 0.1;
        } else if ((P_x + I_x + D_x) < -0.1) {
            vel_msg.linear.x = -0.1;
            integral_x += (-0.1 - (P_x + I_x + D_x)) * 0.1;
        } else {
            vel_msg.linear.x = (P_x + I_x + D_x);
        }
        prev_error_x = error_x;

        if (approximatelyEqual(error_x, 0.0, 0.05) && approximatelyEqual(error_y, 0.0, 0.05)) {
            vel_msg.linear.x = 0.0;
            vel_msg.linear.y = 0.0;
            out_of_reach = false;
        }
        
        (*baseVelPublisher).publish(vel_msg);
        ros::spinOnce();
        rate.sleep();
    } while(out_of_reach);    
    cout << "Base after adjustment: " << endl;
    cout << "Base X: " << base_x << "\tBase y: " << base_y << endl;
    cout << "===========================================" << endl << endl;
}

void gripperOpen(ros::Publisher* gripperPosPublisher) {
    ros::Rate rate(50);
    const int TOTAL_GRIPPER_JOINTS = 2;
    brics_actuator::JointPositions gripper_command;
    vector <brics_actuator::JointValue> gripperJointPositions;
    gripperJointPositions.resize(TOTAL_GRIPPER_JOINTS);
    stringstream jointName;
    gripperJointPositions[0].joint_uri = "gripper_finger_joint_l";
    gripperJointPositions[0].value = 0.0115;
    gripperJointPositions[0].unit = boost::units::to_string(boost::units::si::meter);
    gripperJointPositions[1].joint_uri = "gripper_finger_joint_r";
    gripperJointPositions[1].value = 0.0115;
    gripperJointPositions[1].unit = boost::units::to_string(boost::units::si::meter);
    gripper_command.positions = gripperJointPositions;
    (*gripperPosPublisher).publish(gripper_command);
    ros::spinOnce();
    ros::Duration(3.0).sleep();
    cout << "DONE GRIP OPEN!" << endl;
}

void gripperPos(ros::Publisher* gripperPosPublisher, double space) {
    ros::Rate rate(50);
    const int TOTAL_GRIPPER_JOINTS = 2;
    brics_actuator::JointPositions gripper_command;
    vector <brics_actuator::JointValue> gripperJointPositions;
    gripperJointPositions.resize(TOTAL_GRIPPER_JOINTS);
    stringstream jointName;
    gripperJointPositions[0].joint_uri = "gripper_finger_joint_l";
    gripperJointPositions[0].value = space;
    gripperJointPositions[0].unit = boost::units::to_string(boost::units::si::meter);
    gripperJointPositions[1].joint_uri = "gripper_finger_joint_r";
    gripperJointPositions[1].value = space;
    gripperJointPositions[1].unit = boost::units::to_string(boost::units::si::meter);
    gripper_command.positions = gripperJointPositions;
    (*gripperPosPublisher).publish(gripper_command);
    ros::spinOnce();
    ros::Duration(3.0).sleep();
    cout << "DONE GRIP POSE!" << endl;
}

void moveArm(double joint1, double joint2, double joint3, double joint4, double joint5, ros::Publisher* armPosPublisher) {
    ros::Rate rate(50);
    brics_actuator::JointPositions arm_command;
    vector <brics_actuator::JointValue> armJointPositions;
    armJointPositions.resize(5);
    stringstream jointName;
    double theta[5] = {0};
    theta[0] = - joint1 + 169 * PI / 180; // from -169 to 169
    theta[1] = joint2 + 155 * PI / 180; // from -6 to -154
    theta[2] = joint3 - 146 * PI / 180; // from -141 to 145
    theta[3] = joint4 + 102.5 * PI / 180; // from -93 to 93 (can be reduce to -90 to 90 easier implementation)
    theta[4] = - joint5 + 167.5 * PI / 180; // - 155 to 160

    for (int i = 0; i < 5; i++) {
        jointName.str("");
        jointName << "arm_joint_" << (i + 1);
        armJointPositions[i].joint_uri = jointName.str();
        armJointPositions[i].value = theta[i];
        armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);
    }
    arm_command.positions = armJointPositions;
	(*armPosPublisher).publish(arm_command);
    ros::spinOnce();
    ros::Duration(4.5).sleep();
    cout << "DONE MOVE ARM!" << endl;
}

void pickOnFloor(double x, double y, tf2_ros::Buffer* tfBuffer, ros::Publisher* baseVelPublisher, ros::Publisher* armPosPublisher, ros::Publisher* gripperPosPublisher) {
    ros::Rate rate(50);
    geometry_msgs::TransformStamped transformStamped;
    try {
        transformStamped = (*tfBuffer).lookupTransform("odom", "arm_link_0", ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    double base_x = transformStamped.transform.translation.x - 0.143;
    double base_y = transformStamped.transform.translation.y;
    double base_z = 0.111;
    double theta[5] = {0};
    baseAdjustment(base_x, base_y, x, y, 0.21, 90, 1, theta[0], tfBuffer, baseVelPublisher);
    double zw = 0.0 - base_z - d1 + d5;
    gripperOpen(gripperPosPublisher);

    try {
        transformStamped = (*tfBuffer).lookupTransform("odom", "arm_link_0", ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    base_x = transformStamped.transform.translation.x - 0.143;
    base_y = transformStamped.transform.translation.y;
    theta[0] = atan2(y + base_y, x + base_x);
    double p = sqrt(pow(y + base_y, 2) + pow(x + base_x, 2)) - a1 * cos(theta[0]);

    inverseKinematics(0.29, 0.135, 0, 0, theta[1], theta[2], theta[3], theta[4], true);
    moveArm(theta[0], theta[1], theta[2], theta[3], theta[4], armPosPublisher);

    inverseKinematics(zw, p, 0, 90, theta[1], theta[2], theta[3], theta[4], true);
    moveArm(theta[0], theta[1], theta[2], theta[3], theta[4], armPosPublisher);
    gripperPos(gripperPosPublisher, 0.0);
    
    inverseKinematics(0.29, 0.135, 0, 0, theta[1], theta[2], theta[3], theta[4], true);
    moveArm(theta[0], theta[1], theta[2], theta[3], theta[4], armPosPublisher);
    cout << "DONE PICK!" << endl;
}

void releaseOnFloor(double x, double y, tf2_ros::Buffer* tfBuffer, ros::Publisher* baseVelPublisher, ros::Publisher* armPosPublisher, ros::Publisher* gripperPosPublisher) {
    ros::Rate rate(50);
    geometry_msgs::TransformStamped transformStamped;
    try {
        transformStamped = (*tfBuffer).lookupTransform("odom", "arm_link_0", ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    double base_x = transformStamped.transform.translation.x - 0.143;
    double base_y = transformStamped.transform.translation.y;
    double base_z = 0.111;
    double theta[5] = {0};
    baseAdjustment(base_x, base_y, x, y, 0.21, 90, 1, theta[0], tfBuffer, baseVelPublisher);
    double zw = 0.0 - base_z - d1 + d5;

    try {
        transformStamped = (*tfBuffer).lookupTransform("odom", "arm_link_0", ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    base_x = transformStamped.transform.translation.x - 0.143;
    base_y = transformStamped.transform.translation.y;
    theta[0] = atan2(y + base_y, x + base_x);
    double p = sqrt(pow(y + base_y, 2) + pow(x + base_x, 2)) - a1 * cos(theta[0]);

    inverseKinematics(0.29, 0.135, 0, 0, theta[1], theta[2], theta[3], theta[4], true);
    moveArm(theta[0], theta[1], theta[2], theta[3], theta[4], armPosPublisher);
    inverseKinematics(zw, p, 0, 90, theta[1], theta[2], theta[3], theta[4], true);
    moveArm(theta[0], theta[1], theta[2], theta[3], theta[4], armPosPublisher);
    gripperOpen(gripperPosPublisher);
    inverseKinematics(0.29, 0.135, 0, 0, theta[1], theta[2], theta[3], theta[4], true);
    moveArm(theta[0], theta[1], theta[2], theta[3], theta[4], armPosPublisher);
    cout << "DONE RELEASE!" << endl;
}

void getHomePos(tf2_ros::Buffer* tfBuffer, double& home_x, double& home_y) {
    ros::Rate rate(50);
    geometry_msgs::TransformStamped transformStamped;
    try {
        transformStamped = (*tfBuffer).lookupTransform("odom", "arm_link_0", ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    home_x = transformStamped.transform.translation.x;
    home_y = transformStamped.transform.translation.y;
    ros::spinOnce();
}
void returnHome(double home_x, double home_y, tf2_ros::Buffer* tfBuffer, ros::Publisher* baseVelPublisher) {
    ros::Rate rate(50);
    geometry_msgs::Twist vel_msg;
    geometry_msgs::TransformStamped transformStamped;
    bool out_of_reach = true;
    double base_x, base_y;
    double kp_y, ki_y, kd_y, dt, integral_y, prev_error_y;
    double kp_x, ki_x, kd_x, integral_x, prev_error_x;
    dt = 0.02;
    integral_y = 0.0;
    prev_error_y = 0.0;
    integral_x = 0.0;
    prev_error_x = 0.0;
    kp_y = 0.3;
    ki_y = 0.05;
    kd_y = 0.01;
    kp_x = 0.3;
    ki_x = 0.05;
    kd_x = 0.01;
    do {
        try {
            transformStamped = (*tfBuffer).lookupTransform("odom", "arm_link_0", ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        base_x = transformStamped.transform.translation.x;
        base_y = transformStamped.transform.translation.y;

        double error_y = home_y - base_y;
        double P_y = error_y * kp_y;
        integral_y += error_y * dt;
        double I_y = integral_y * ki_y;
        double D_y = kd_y * (error_y - prev_error_y) / dt;        
        if ((P_y + I_y + D_y) > 0.1) {
            vel_msg.linear.y = 0.1;
            integral_y += (0.1 - (P_y + I_y + D_y)) * 0.1;
        } else if ((P_y + I_y + D_y) < -0.1) {
            vel_msg.linear.y = -0.1;
            integral_y += (-0.1 - (P_y + I_y + D_y)) * 0.1;
        } else {
            vel_msg.linear.y = (P_y + I_y + D_y);
        }
        prev_error_y = error_y;

        double error_x = home_x - base_x;
        double P_x = error_x * kp_x;
        integral_x += error_x * dt;
        double I_x = integral_x * ki_x;
        double D_x = kd_x * (error_x - prev_error_x) / dt;
        if ((P_x + I_x + D_x) > 0.1) {
            vel_msg.linear.x = 0.1;
            integral_x += (0.1 - (P_x + I_x + D_x)) * 0.1;
        } else if ((P_x + I_x + D_x) < -0.1) {
            vel_msg.linear.x = -0.1;
            integral_x += (-0.1 - (P_x + I_x + D_x)) * 0.1;
        } else {
            vel_msg.linear.x = (P_x + I_x + D_x);
        }
        prev_error_x = error_x;

        if (approximatelyEqual(error_x, 0.0, 0.003) && approximatelyEqual(error_y, 0.0, 0.003)) {
            vel_msg.linear.x = 0.0;
            vel_msg.linear.y = 0.0;
            out_of_reach = false;
        }
        
        (*baseVelPublisher).publish(vel_msg);
        ros::spinOnce();
        rate.sleep();
    } while(out_of_reach);
    cout << "RETURNED HOME" << endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "kinematics");
    ros::NodeHandle nh;

    ros::Publisher armPosPublisher;
    ros::Publisher baseVelPublisher;
    ros::Publisher gripperPosPublisher;
    ros::Rate rate(50);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    armPosPublisher = nh.advertise<brics_actuator::JointPositions > ("arm_1/arm_controller/position_command", 1);
    gripperPosPublisher = nh.advertise<brics_actuator::JointPositions > ("arm_1/gripper_controller/position_command", 1);
    baseVelPublisher = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // double base_x = 0.0;
    // double base_y = 0.0;
    // double base_z = 0.0;
    // double target_x, target_y, target_z, target_p, roll, pitch;
    // double theta[5] = {0};
    
    bool do_one = false;
    double home_x, home_y;
    
    while(nh.ok()) {
        // brics_actuator::JointPositions arm_command;
        // vector <brics_actuator::JointValue> armJointPositions;
		// armJointPositions.resize(TOTAL_ARM_JOINTS);
        // stringstream jointName;
        // geometry_msgs::TransformStamped transformStamped;
        // try {
        //     transformStamped = tfBuffer.lookupTransform("odom", "arm_link_0", ros::Time(0));
        // }
        // catch (tf2::TransformException &ex) {
        //     ROS_WARN("%s", ex.what());
        //     ros::Duration(1.0).sleep();
        //     continue;
        // }
        // base_x = transformStamped.transform.translation.x - 0.143;
        // base_y = transformStamped.transform.translation.y;
        // base_z = transformStamped.transform.translation.z + 0.06; //calibrate to make z = 0 ~ floor
        // cout << "Base Position (before adjustment):" << std::endl;
        // cout << "Base X: " << base_x << "   Base Y: " << base_y << "   Base Z: " << base_z << endl;
        // cout << "===========================================" << endl << endl;

        // int p_type = 1;
                
        // inputTarget(roll, pitch, target_z, target_p, target_x, target_y, base_z, p_type);
        // double zw = target_z - base_z - d1 + d5 * sin(degToRad(pitch));

        // baseAdjustment(base_x, base_y, target_x, target_y, target_p, pitch, p_type, theta[0], &tfBuffer, &baseVelPublisher);

        // inverseKinematics(zw, target_p, roll, pitch, theta[1], theta[2], theta[3], theta[4], true);
        // cout << "Result of inverse kinematics: " << endl;
        // for (int i = 0; i < TOTAL_ARM_JOINTS; i++) {
        //     cout << "Theta " << i + 1 << " = " << radToDeg(theta[i]) << " degrees" << endl;
        // }
        // cout << "===========================================" << endl << endl;

        // theta[0] = - theta[0] + 169 * PI / 180; // from -169 to 169
        // theta[1] = theta[1] + 155 * PI / 180; // from -6 to -154
        // theta[2] = theta[2] - 146 * PI / 180; // from -141 to 145
        // theta[3] = theta[3] + 102.5 * PI / 180; // from -93 to 93 (can be reduce to -90 to 90 easier implementation)
        // theta[4] = - theta[4] + 167.5 * PI / 180; // - 155 to 160

        // for (int i = 0; i < TOTAL_ARM_JOINTS; i++) {
        //     jointName.str("");
        //     jointName << "arm_joint_" << (i + 1);
		// 	armJointPositions[i].joint_uri = jointName.str();
        //     armJointPositions[i].value = theta[i];
		// 	armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);
		// }
		// cout << "sending command ..." << endl;
		// arm_command.positions = armJointPositions;
		// armPosPublisher.publish(arm_command);
        // gripperOpen(&gripperPosPublisher);
		// cout << "--------------------" << endl;
        if (!do_one) {
            getHomePos(&tfBuffer, home_x, home_y);
            pickOnFloor(0.3, -0.4, &tfBuffer, &baseVelPublisher, &armPosPublisher, &gripperPosPublisher);
            releaseOnFloor(0.7, 0.5, &tfBuffer, &baseVelPublisher, &armPosPublisher, &gripperPosPublisher);
            pickOnFloor(0.7, 0.5, &tfBuffer, &baseVelPublisher, &armPosPublisher, &gripperPosPublisher);
            releaseOnFloor(0.3, -0.4, &tfBuffer, &baseVelPublisher, &armPosPublisher, &gripperPosPublisher);
            returnHome(home_x, home_y, &tfBuffer, &baseVelPublisher);
            do_one = true;
            cout << "DONE ALL!" << endl;
        }        
		ros::spinOnce();
		rate.sleep();
    }
    return 0;
}