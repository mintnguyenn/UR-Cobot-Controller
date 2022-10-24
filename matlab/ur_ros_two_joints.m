%%
clear; clc;

q1 = [0, -pi/2, 0, -pi/2, 0, 0];
q2 = [-pi/4, -pi/3, pi/2, 0, 0, 0];
qq2 = [-2.298648428654857,-2.117012324259335,-2.001421505380947,-2.054811788887437,-3.872453542649573, 0];
qq1 = [-2.12305, -1.22493, -1.61512, 1.48271, -6.1051, 0];
qqq = wrapToPi([5.9434    5.2201    1.8738    0.7600    1.5708    2.8018]);
%
jointStateSubscriber = rossubscriber('joint_states','sensor_msgs/JointState');
pause(1);
[client, goal] = rosactionclient('/scaled_pos_joint_traj_controller/follow_joint_trajectory');
goal.Trajectory.JointNames = {'shoulder_pan_joint','shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};
goal.Trajectory.Header.Seq = 1;
goal.GoalTimeTolerance = rosduration(0.05);

startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
pause(0.3);

currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; % Note the default order of the joints is 3,2,1,4,5,6
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];
goal.Trajectory.Header.Stamp = rostime('Now','system');

startJointSend.Positions = currentJointState_123456;
startJointSend.TimeFromStart = rosduration(0);     

endJointSend.Positions = q1;
endJointSend.TimeFromStart = rosduration(3); % Duration second, this is how many seconds the movement will take

goal.Trajectory.Points = [startJointSend; endJointSend];

goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(0.5); % This allows for the time taken to send the message. If the network is fast, this could be reduced.
sendGoal(client,goal);
%%
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; % Note the default order of the joints is 3,2,1,4,5,6
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];
goal.Trajectory.Header.Stamp = rostime('Now','system');

startJointSend.Positions = currentJointState_123456;
startJointSend.TimeFromStart = rosduration(0);     

endJointSend.Positions = q1;
endJointSend.TimeFromStart = rosduration(2); % Duration second, this is how many seconds the movement will take

goal.Trajectory.Points = [startJointSend; endJointSend];
goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(0.5); % This allows for the time taken to send the message. If the network is fast, this could be reduced.

sendGoal(client,goal);
