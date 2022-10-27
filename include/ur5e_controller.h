#ifndef UR5E_CONTROLLER_H
#define UR5E_CONTROLLER_H

#include <iostream>
#include <sstream>
#include <fstream>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <angles/angles.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;

class Manipulator_Controller
{
public:
    Manipulator_Controller();
    ~Manipulator_Controller();

    void trajectoryBetween2Points(std::vector<double> start_point, std::vector<double> end_point);

    void trajectoryFromArray(std::vector<std::vector<double>> array);

private:
    Client* client_;
    control_msgs::FollowJointTrajectoryGoal goal_;
};

#endif
