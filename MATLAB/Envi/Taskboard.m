classdef Taskboard < handle
    properties
        mainboard;
        key;
        cable;
        battCoin;
        battAA1;
        battAA2;
        battCap;
    end
    
    properties (Access = private)
        keyOffset = transl(-0.08861,0.0452,-0.0037) * rpy2tr(0,0,0);
        cableOffset = transl(-0.005044,-0.022093,0.02124) * rpy2tr(0,0,pi);
        battCapOffset = transl(-0.03674,0.0397,0.007386) * rpy2tr(0,0,0);
        battCoinOffset = transl(0.06404,-0.03233,0.001) * rpy2tr(0,0,0);
        battAA1Offset = transl(-0.04495,0.047206,0.0012084) * rpy2tr(0,0,0);
        battAA2Offset = transl(-0.02127,0.032206,0.0012084) * rpy2tr(0,0,pi);
    end
    
    methods
        function self = Taskboard(spawnPose)
            self.mainboard = Environment("Taskboard.ply", spawnPose);
            self.key = Environment("TaskboardKey.ply", spawnPose * self.keyOffset);
            self.cable = Environment("EthernetCable.ply", spawnPose * self.cableOffset);
            self.battCap = Environment("AABatteryCap.ply", spawnPose * self.battCapOffset);
            self.battCoin = Environment("CoinBattery.ply", spawnPose * self.battCoinOffset);
            self.battAA1 = Environment("AABattery.ply", spawnPose * self.battAA1Offset);
            self.battAA2 = Environment("AABattery.ply", spawnPose * self.battAA2Offset);
        end
        
        function PlotTaskboard(self)
            self.mainboard.PlotModel();
            self.key.PlotModel();
            self.cable.PlotModel();
            self.battCap.PlotModel();
            self.battCoin.PlotModel();
            self.battAA1.PlotModel();
            self.battAA2.PlotModel();
        end
        
        function MoveTaskboard(self, goalPose)
            self.mainboard.MoveModel(goalPose);
            self.key.MoveModel(goalPose);
            self.cable.MoveModel(goalPose);
            self.battCap.MoveModel(goalPose);
            self.battCoin.MoveModel(goalPose);
            self.battAA1.MoveModel(goalPose);
            self.battAA2.MoveModel(goalPose);
        end
        
        
    end
end


