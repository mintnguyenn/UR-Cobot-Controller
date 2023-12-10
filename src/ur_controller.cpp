#include "../include/ur_controller.h"

std::vector<std::vector<double>> input_data_joint_space(std::string file_name)
{
    std::vector<std::vector<double>> result;
    result.clear();

    std::ifstream infile;
    infile.open(file_name.data(), std::ios::in);
    assert(infile.is_open());

    while (infile)
    {
        std::string s;
        std::getline(infile, s);
        std::istringstream is(s);
        std::vector<double> join_space;
        join_space.resize(6);
        is >> join_space.at(0) >> join_space.at(1) >> join_space.at(2) >> join_space.at(3) >> join_space.at(4) >> join_space.at(5);
        result.push_back(join_space);
    }
    result.pop_back();

    return result;
}

trajectory_msgs::JointTrajectoryPoint initialiseTrajectoryPoint(const std::vector<double> joint_space, double duration)
{
    trajectory_msgs::JointTrajectoryPoint points;

    points.positions.resize(6);
    points.velocities.resize(6);
    points.accelerations.resize(6);
    for (unsigned int i = 0; i < 6; i++)
    {
        points.positions[i] = joint_space.at(i);
        points.velocities[i] = 0.0;
        points.accelerations[i] = 0.0;
    }

    points.time_from_start = ros::Duration(duration);

    return points;
}

UR_Controller::UR_Controller()
{
    client_ = new Client("/eff_joint_traj_controller/follow_joint_trajectory", true);
    // client_ = new Client("/pos_joint_traj_controller/follow_joint_trajectory", true);

    ROS_INFO("Waiting for action server to start.");

    // Wait for the action server to start
    client_->waitForServer(); // Will wait for infinite time

    ROS_INFO("Action server started, sending goal.");
}

UR_Controller::~UR_Controller() {}

void UR_Controller::run(std::vector<double> point)
{
    //
    goal_.trajectory.header.stamp = ros::Time::now() + ros::Duration(1);

    // Initialise joint names
    goal_.trajectory.joint_names.resize(6);
    goal_.trajectory.joint_names[0] = "shoulder_pan_joint";
    goal_.trajectory.joint_names[1] = "shoulder_lift_joint";
    goal_.trajectory.joint_names[2] = "elbow_joint";
    goal_.trajectory.joint_names[3] = "wrist_1_joint";
    goal_.trajectory.joint_names[4] = "wrist_2_joint";
    goal_.trajectory.joint_names[5] = "wrist_3_joint";

    goal_.trajectory.points.resize(1);
    goal_.trajectory.points.at(0) = initialiseTrajectoryPoint(point, 5.0); /* Take 5 seconds for this movement. Can be modified */

    client_->sendGoal(goal_);

    while (client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED && ros::ok())
    {
        client_->waitForResult(ros::Duration(1));

        ROS_INFO("Current State: %s", client_->getState().toString().c_str());
        sleep(2);
    }
    ROS_INFO("Action ended!");
}

void UR_Controller::run(std::vector<double> start_point, std::vector<double> end_point)
{
    //
    goal_.trajectory.header.stamp = ros::Time::now() + ros::Duration(1);

    // Initialise joint names
    goal_.trajectory.joint_names.resize(6);
    goal_.trajectory.joint_names[0] = "shoulder_pan_joint";
    goal_.trajectory.joint_names[1] = "shoulder_lift_joint";
    goal_.trajectory.joint_names[2] = "elbow_joint";
    goal_.trajectory.joint_names[3] = "wrist_1_joint";
    goal_.trajectory.joint_names[4] = "wrist_2_joint";
    goal_.trajectory.joint_names[5] = "wrist_3_joint";

    goal_.trajectory.points.resize(2);
    goal_.trajectory.points.at(0) = initialiseTrajectoryPoint(start_point, 8.0);
    goal_.trajectory.points.at(1) = initialiseTrajectoryPoint(end_point, 15.0);

    client_->sendGoal(goal_);

    while (client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED && ros::ok())
    {
        client_->waitForResult(ros::Duration(1));

        ROS_INFO("Current State: %s", client_->getState().toString().c_str());
        sleep(2);
    }
    ROS_INFO("Action ended!");
}

void UR_Controller::run(std::vector<std::vector<double>> trajectory)
{
    //
    goal_.trajectory.header.stamp = ros::Time::now() + ros::Duration(1);

    // Initialise joint names
    goal_.trajectory.joint_names.resize(6);
    goal_.trajectory.joint_names[0] = "shoulder_pan_joint";
    goal_.trajectory.joint_names[1] = "shoulder_lift_joint";
    goal_.trajectory.joint_names[2] = "elbow_joint";
    goal_.trajectory.joint_names[3] = "wrist_1_joint";
    goal_.trajectory.joint_names[4] = "wrist_2_joint";
    goal_.trajectory.joint_names[5] = "wrist_3_joint";

    goal_.trajectory.points.resize(trajectory.size());
    for (unsigned int i = 0; i < trajectory.size(); i++)
    {
        double time;
        time = 4 + (i * 0.11);
        goal_.trajectory.points.at(i) = initialiseTrajectoryPoint(trajectory.at(i), time);
    }

    client_->sendGoal(goal_);

    while (client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED && ros::ok())
    {
        client_->waitForResult(ros::Duration(1));

        ROS_INFO("Current State: %s", client_->getState().toString().c_str());
        sleep(2);
    }
    ROS_INFO("Action ended!");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ur_controller");

    ros::NodeHandle nh("~");

    // Retrieve the 'mode' parameter from the Parameter Server
    std::string mode;
    nh.param("mode", mode, std::string("initialise"));

    ROS_INFO("Mode parameter set to: %s", mode.c_str());

    std::shared_ptr<UR_Controller> controller(new UR_Controller());

    if (mode.compare("home") == 0)
    {
        ROS_INFO("Mode HOME");
        controller->run({0, -M_PI/2, 0, -M_PI / 2, 0, 0});
    }

    else if (mode.compare("initialise") == 0)
    {
        std::vector<double> start_point{0, -M_PI / 2, 0, -M_PI / 2, 0, 0};
        std::vector<double> end_point{-M_PI / 4, -M_PI / 3, M_PI / 2, 0, 0, 0};

        // std::vector<double> start_point{-3.0337, -2.3005, -1.9821, -6.1475, -4.5623, 0};
        // std::vector<double> end_point{-3.0138, -2.2941, -1.9948, -6.1407, -4.5665, 0};
        controller->run(start_point, end_point);
        ROS_INFO("Mode INITIALISE");
    }

    else if (mode.compare("motion") == 0)
    {
        // std::vector<std::vector<double>> motion = input_data_joint_space("/home/mintnguyen/catkin_workspace/NRMDTS_ws/src/Manipulator_Controller/data/motion_1.txt");

        // std::cout << motion.size() << std::endl;
        // controller->trajectoryFromArray(motion);
        ROS_INFO("Mode MOTION");
    }
    ros::spin();

    /**
     * Let's cleanup everything, shutdown ros and join the thread
     */
    ros::shutdown();

    nh.deleteParam("mode");

    return 0;
}