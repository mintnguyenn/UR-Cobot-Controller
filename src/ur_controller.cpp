#include "ur_controller.h"

std::vector<std::vector<double>> input_data_joint_spacee(std::string file_name)
{
    std::vector<std::vector<double>> result;
    result.clear();

    std::ifstream infile;
    infile.open(file_name.data(), std::ios::in);
    assert(infile.is_open());

    while (infile){
        std::string s;
        std::getline(infile, s);
        std::istringstream is(s);
        std::vector<double> join_space;
        join_space.resize(6);
        is >> join_space.at(0) >> join_space.at(1) >> join_space.at(2) >> join_space.at(3) >> join_space.at(4) >> join_space.at(5);
        // wrapToPi(join_space);
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

Manipulator_Controller::Manipulator_Controller(){
    // client_ = new Client("/scaled_pos_joint_traj_controller/follow_joint_trajectory", true);
    client_ = new Client("/pos_joint_traj_controller/follow_joint_trajectory", true);

    ROS_INFO("Waiting for action server to start.");

    // Wait for the action server to start
    client_->waitForServer(); // Will wait for infinite time

    ROS_INFO("Action server started, sending goal.");
}

Manipulator_Controller::~Manipulator_Controller() {}

void Manipulator_Controller::trajectoryBetween2Points(std::vector<double> start_point, std::vector<double> end_point){
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
    goal_.trajectory.points.at(0) = initialiseTrajectoryPoint(start_point, 4.0);
    goal_.trajectory.points.at(1) = initialiseTrajectoryPoint(end_point, 8.0);

    client_->sendGoal(goal_);

    while (client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED && ros::ok())
    {
        client_->waitForResult(ros::Duration(1));

        ROS_INFO("Current State: %s", client_->getState().toString().c_str());
        sleep(2);
    }
    ROS_INFO("Action ended!");
}

void Manipulator_Controller::trajectoryFromArray(std::vector<std::vector<double>> array){
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

    goal_.trajectory.points.resize(array.size());
    for (unsigned int i = 0; i < array.size(); i++){
        // wrapToPi(array.at(i));
        double time;
        time = 4 + (i * 0.1);
        goal_.trajectory.points.at(i) = initialiseTrajectoryPoint(array.at(i), time);
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
    std::string mode;
    nh.param<std::string>("mode", mode, "0"); // Run as basic mode

    std::shared_ptr<Manipulator_Controller> controller(new Manipulator_Controller());

    if (mode.compare("testing") == 0){
        std::vector<double> start_point{2 * M_PI, -M_PI / 2, 0, -M_PI / 2, 0, 0};
        std::vector<double> end_point{-M_PI / 4, -M_PI / 3, M_PI / 2, 0, 0, 0};
        controller->trajectoryBetween2Points(start_point, end_point);
    }

    std::cout << mode << std::endl;

    std::vector<std::vector<double>> motion;
    motion = input_data_joint_spacee("/home/mintnguyen/catkin_workspace/NRMDTS_ws/src/Manipulator_Controller/data/tmech22.txt");

    // std::cout << motion.size() << std::endl;
    //   controller->trajectoryFromArray(motion);

    ros::spin();

    /**
     * Let's cleanup everything, shutdown ros and join the thread
     */
    ros::shutdown();

    return 0;
}