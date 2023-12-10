#ifndef UR_CONTROLLER_H
#define UR_CONTROLLER_H

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

class UR_Controller
{

public:
    /* Default constructor */
    UR_Controller();

    /* Default destructor */
    ~UR_Controller();

    /**
     * Move the robot to a given joint space
     * 
     * @param point: a vector of 6 doubles value representing the joint space
    */
    void run(std::vector<double> point);

    /**
     * Move the robot from a start point to an end point
     * 
     * @param start_point: a vector of 6 doubles value representing the start point
     * @param end_point: a vector of 6 doubles value representing the end point    
    */
    void run(std::vector<double> start_point, std::vector<double> end_point);

    /**
     * Move the robot along a given trajectory
     * 
     * @param array: a vector of vector of 6 doubles value representing the trajectory    
    */
    void run(std::vector<std::vector<double>> trajectory);

private:
    Client* client_;
    control_msgs::FollowJointTrajectoryGoal goal_;
};

#endif
