%roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=169.254.16.251 kinematics_config:=$(rospack find ur_calibration)/ur5e_calibration.yaml

% This script operates the real UR by sending a joint array to the robot.

clear;
pause(10);
load icra23_1.mat

jointStateSubscriber = rossubscriber('joint_states','sensor_msgs/JointState');
pause(1);
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; % Note the default order of the joints is 3,2,1,4,5,6
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];

[client, goal] = rosactionclient('/scaled_pos_joint_traj_controller/follow_joint_trajectory');
goal.Trajectory.JointNames = {'shoulder_pan_joint','shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};
goal.Trajectory.Header.Seq = 1;
goal.Trajectory.Header.Stamp = rostime('Now','system');
goal.GoalTimeTolerance = rosduration(0.05);

startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');

startJointSend.Positions = currentJointState_123456;
startJointSend.TimeFromStart = rosduration(0);
joint_trajectory_send = [startJointSend];

for i=1:size(jointConfigArray,1)
    midJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
    midJointSend.Positions = jointConfigArray(i, 1:6);%-(2*pi);
    midJointSend.Positions(6) = 0;
    midJointSend.TimeFromStart = rosduration((i/10));
    joint_trajectory_send = [joint_trajectory_send; midJointSend];
end

goal.Trajectory.Points = joint_trajectory_send;

goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(0.5);
sendGoal(client,goal);


