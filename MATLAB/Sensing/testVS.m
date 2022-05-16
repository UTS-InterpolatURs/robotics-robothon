clc;
clear;
clf;
addpath '../UR10e/'
addpath '../'
      
robot = UTS_UR10(); 
while true
    obj = urVisualServoing(robot);
    try
        obj.subscribeVS();
        disp(obj.getVsTr);
    catch
        disp('nah')
    end
end