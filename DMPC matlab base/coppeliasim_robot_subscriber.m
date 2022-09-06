function [X_robot] = coppeliasim_robot_subscriber(tb3_posesub_i)

% posesub = rossubscriber('/tb3_0/odom');
posedata = receive(tb3_posesub_i,10);

% Position of agent i from gazebo
posi_x = posedata.Linear.X;
posi_y = posedata.Linear.Y;

% Velocity of agent i from gazebo
vel_x = posedata.Angular.X;
vel_y = posedata.Angular.Y;

% states vector
X_robot = [posi_x; posi_y; vel_x; vel_y];