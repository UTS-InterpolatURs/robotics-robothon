pub = rospublisher('/desired_joint_states', 'sensor_msgs/JointState');
msg = rosmessage(pub);

msg.Name = {'left_jaw', 'right_jaw'};
msg.Position = [0.15,-0.15];

send(pub,msg);