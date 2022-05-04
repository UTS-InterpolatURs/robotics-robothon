clear; clc
try 
    rosshutdown();
    rosinit();
    addpath '../UR10e/'
    addpath '../'
    addpath '/bagfile'
    robot = UTS_UR10;
end

while true
     
    q1 = deg2rad([0 -45 45 -90 -90 0]);
    
    q2 = [0.0000 -0.5946 -5.1732 4.1970 4.7124 6.2832];
    qmatrix = [q1;q2];
    
    EE_current = robot.model.fkine(q1);
    EE_next = robot.model.fkine(q2);
    
    rpyEE_current = tr2rpy(EE_current);
    quaternion = angle2quat(rpyEE_current(1,3),rpyEE_current(1,2),rpyEE_current(1,1));
    
    vSet = viewSet;
    
    for i = 1:length(qmatrix(:,1))
        EE_tr = robot.model.fkine(qmatrix(i,1:6));
        
        vSet = addView(vSet,i,'Orientation',EE_tr(1:3,1:3),'Location',[EE_tr(1,4),EE_tr(2,4),EE_tr(3,4)]);
    end
    
    try node = ros.Node('/TransSender'); end
    
    pub = ros.Publisher(node,'/trEE','geometry_msgs/PoseArray');
    
    msg = rosmessage(pub);
    msg.Poses = vSet.Views;
    
%     msg.Transform.Rotation.X = quaternion(1,1);
%     msg.Transform.Rotation.Y = quaternion(1,2);
%     msg.Transform.Rotation.Z = quaternion(1,3);
%     msg.Transform.Rotation.W = quaternion(1,4);
    
    msg.Header.Seq = 1;
    msg.Header.FrameId = 'camera_link';
    msg.Header.Stamp = rostime('Now','system');
    send(pub,msg);
    pause(2);
    
%     
%     msg.Header.Seq = 1;
%     msg.Header.FrameId = 'camera_link';
%     msg.Header.Stamp = rostime('Now','system');
%     send(pub,msg);
%     pause(2);
end