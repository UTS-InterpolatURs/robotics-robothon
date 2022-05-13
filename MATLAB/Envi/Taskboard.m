classdef Taskboard < handle
    properties
        mainboard;
        key;
        cable;
        battCoin;
        battAA;
        battCap;
    end
    
    properties (Access = private)
        keyOffset = transl(-0.08861,0.0452,-0.0037) * rpy2tr(0,0,0);
        cableOffset;
        battCoinOffset;
        battAAOffset;
        battCapOffset;
    end
    
    methods
        function self = Taskboard(spawnPose)
            self.mainboard = Environment("Taskboard.ply", spawnPose);
            self.key = Environment("TaskboardKey.ply", spawnPose * self.keyOffset);
        end
        
        function PlotTaskboard(self)
            self.mainboard.PlotModel();
            self.key.PlotModel();
        end  
    end
end


