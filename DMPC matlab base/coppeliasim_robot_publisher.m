function coppeliasim_robot_publisher(vel_i_sample, tb3_pub_i)

% tb0_pub = rospublisher("/tb3_0/cmd_vel","geometry_msgs/Twist");
msg = rosmessage(tb3_pub_i);

for i = length(vel_i_sample)
    xdot = vel_i_sample(1,i);
    ydot = vel_i_sample(2,i);
    
    
    msg.Linear.X = xdot;
    msg.Linear.Y = ydot;
    msg.Linear.Z = 0;
    msg.Angular.X = 0;
    msg.Angular.Y = 0;
    msg.Angular.Z = 0;
    send(tb3_pub_i,msg);
    waitfor(rosrate(1/0.01));
end

end
