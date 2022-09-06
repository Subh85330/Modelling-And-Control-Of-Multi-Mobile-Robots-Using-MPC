function robot_publisher(vel_i_sample, tb3_pub_i, tb3_posesub_i)

% tb0_pub = rospublisher("/tb3_0/cmd_vel","geometry_msgs/Twist");
msg = rosmessage(tb3_pub_i);

for i = length(vel_i_sample)
    xdot = vel_i_sample(1,i);
    ydot = vel_i_sample(2,i);
    
    vel_tb3_0 = sqrt(xdot^2 + ydot^2);
    vel_ang_tb3_0 = atan2(ydot, xdot);% + 360*(ydot<0);
    [~, theta_tb3_0] = robot_subscriber(tb3_posesub_i);
    ang_vel_tb3_0 = 0.1*(vel_ang_tb3_0 - theta_tb3_0);
    
    msg.Linear.X = vel_tb3_0;
    msg.Linear.Y = 0;
    msg.Linear.Z = 0;
    msg.Angular.X = 0;
    msg.Angular.Y = 0;
    msg.Angular.Z = ang_vel_tb3_0;
    send(tb3_pub_i,msg);
end

end
