%%
clear; clc;

q  = [-3.0057   -2.2892   -2.0022    0.1346    1.7001         0]; % Start motion 1
q1 = [-3.0450   -2.3094   -1.9835    0.1544    1.7838         0]; % Start reconf 1
q2 = [-0.5195   -0.5625    1.1903   -1.3220   -0.7597         0]; % Start motion 2
q3 = [-0.5224   -0.6149    1.2887   -1.4845   -0.7378         0]; % Start reconf 2
qI = [-0.1944   -0.8416    1.5644    3.9896   -1.5708         0];

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

endJointSend.Positions = q;
endJointSend.TimeFromStart = rosduration(5); % Duration second, this is how many seconds the movement will take

goal.Trajectory.Points = [startJointSend; endJointSend];

goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(0.5); % This allows for the time taken to send the message. If the network is fast, this could be reduced.
sendGoal(client,goal);
%%
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; % Note the default order of the joints is 3,2,1,4,5,6
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];
goal.Trajectory.Header.Stamp = rostime('Now','system');

startJointSend.Positions = currentJointState_123456;
startJointSend.TimeFromStart = rosduration(0);     

endJointSend.Positions = q;
endJointSend.TimeFromStart = rosduration(2); % Duration second, this is how many seconds the movement will take

goal.Trajectory.Points = [startJointSend; endJointSend];
goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(0.5); % This allows for the time taken to send the message. If the network is fast, this could be reduced.

sendGoal(client,goal);
