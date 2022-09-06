function [X_robot, theta_z] = robot_subscriber(tb3_posesub_i)

% posesub = rossubscriber('/tb3_0/odom');
posedata = receive(tb3_posesub_i,10);

% Position of agent i from gazebo
posi_x = posedata.Pose.Pose.Position.X;
posi_y = posedata.Pose.Pose.Position.Y;

% Velocity of agent i from gazebo
vel_x = posedata.Twist.Twist.Linear.X;
vel_y = posedata.Twist.Twist.Linear.Y;

% states vector
X_robot = [posi_x; posi_y; vel_x; vel_y];

% Quterain to euler 
quat_X = posedata.Pose.Pose.Orientation.X;
quat_Y = posedata.Pose.Pose.Orientation.Y;
quat_Z = posedata.Pose.Pose.Orientation.Z;
quat_W = posedata.Pose.Pose.Orientation.W;

% quat = [quat_X quat_Y quat_Z quat_W];
% eulZYX = quat2eul(quat,'XYZ');
yaw=atan2((2*quat_W*quat_Z), quat_W*quat_W - quat_Z*quat_Z);
% Orientation of tb3_0
% theta_tb3_0 = eulZYX(1);
theta_z = yaw;