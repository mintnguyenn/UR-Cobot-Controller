classdef ur5e < handle
    properties
        robot;
        qDef  = [0, -pi/3, pi/2, -2*pi/3, -pi/2, 0];
        qHome = [0, -pi/2, 0, -pi/2, 0, 0];

        client;
        goal;
   
    end

    methods
        % Constructor
        function self = ur5e()
            self.createRobot();
            self.robot.plot(self.qDef);
        end

        % Create robot using DH Parameters
        function createRobot(self)
            % robot length values (metres)
            a = [0, -0.42500, -0.39220, 0, 0, 0]';

            d = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996]';

            alpha = [1.570796327, 0, 0, 1.570796327, -1.570796327, 0]';
    
            theta = zeros(6,1);
    
            DH = [theta d a alpha];

            mass = [3.761, 8.058, 2.846, 1.37, 1.3, 0.365];

            center_of_mass = [
                0,     -0.02561, 0.00193
                0.2125, 0,       0.11336
                0.15,   0,       0.0265
                0,     -0.0018,  0.01634
                0,      0.0018,  0.01634
                0,      0,      -0.001159];

            self.robot = SerialLink(DH, ...
                'name', 'UR5e', 'manufacturer', 'Universal Robotics');

            % add the mass data, no inertia available
            links = self.robot.links;
            for i=1:6
                links(i).m = mass(i);
                links(i).r = center_of_mass(i,:);
            end
        end

        % Control robot
        function moveRobot(self, qMatrix)
            steps = size(qMatrix,1);
            for i=1:steps
                q = qMatrix(i,:);
                self.robot.animate(q);
                drawnow();
            end
        end

        %
        function moveRealRobot(self, qJoint)
            jointStateSubscriber = rossubscriber('joint_states','sensor_msgs/JointState');
            pause(1);
            currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; % Note the default order of the joints is 3,2,1,4,5,6
            currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];

            [self.client, self.goal] = rosactionclient('/scaled_pos_joint_traj_controller/follow_joint_trajectory');
            self.goal.Trajectory.JointNames = {'shoulder_pan_joint','shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};
            self.goal.Trajectory.Header.Seq = 1;
            self.goal.Trajectory.Header.Stamp = rostime('Now','system');
            self.goal.GoalTimeTolerance = rosduration(0.05);

            startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
            startJointSend.Positions = currentJointState_123456;
            startJointSend.TimeFromStart = rosduration(0);     
      
            endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
            endJointSend.Positions = qJoint;
            endJointSend.TimeFromStart = rosduration(5); % Duration second, this is how many seconds the movement will take

            self.goal.Trajectory.Points = [startJointSend; endJointSend];
            pause(1);
            self.goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(0.5); % This allows for the time taken to send the message. If the network is fast, this could be reduced.
            sendGoal(self.client,self.goal);
            pause(5.5);
        end

    end

end