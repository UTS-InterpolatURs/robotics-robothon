classdef ColourDetection < handle
    properties
        channel1Min_ = 0;
        channel1Max_ = 0;
        channel2Min_ = 0;
        channel2Max_ = 0;
        channel3Min_ = 0;
        channel3Max_ = 0;
        SliderBW_ = 0;
        I_;
        colour_ = 0;
    end
    
    methods
        function self = ColourDetection(RGB)
            self.I_ = rgb2hsv(RGB);
            self.SliderBW_ = ( (self.I_(:,:,1) >= self.channel1Min_) & (self.I_(:,:,1) <= self.channel1Max_) ) & ...
                (self.I_(:,:,2) >= self.channel2Min_ ) & (self.I_(:,:,2) <= self.channel2Max_) & ...
                (self.I_(:,:,3) >= self.channel3Min_ ) & (self.I_(:,:,3) <= self.channel3Max_);
        end
        function ImportRGB(self,RGB)
            self.I_ = rgb2hsv(RGB);
        end
        
        function SetChannels(self,l1Min,l1Max,l2Min,l2Max,l3Min,l3Max)
            self.channel1Min_ = l1Min;
            self.channel1Max_ = l1Max;
            self.channel2Min_ = l2Min;
            self.channel2Max_ = l2Max;
            self.channel3Min_ = l3Min;
            self.channel3Max_ = l3Max;
            self.SliderBW_ = ( (self.I_(:,:,1) >= self.channel1Min_) & (self.I_(:,:,1) <= self.channel1Max_) ) & ...
                (self.I_(:,:,2) >= self.channel2Min_ ) & (self.I_(:,:,2) <= self.channel2Max_) & ...
                (self.I_(:,:,3) >= self.channel3Min_ ) & (self.I_(:,:,3) <= self.channel3Max_);
        end
        
        function [l1Min,l1Max,l2Min,l2Max,l3Min,l3Max] = GetChannels(self)
             l1Min = self.channel1Min_;
             l1Max = self.channel1Max_;
             l2Min = self.channel2Min_;
             l2Max = self.channel2Max_;
             l3Min = self.channel3Min_;
             l3Max = self.channel3Max_;
        end
        function SetColour(self)
            
            switch self.colour_
                case 2
                    
                    self.channel1Min_ = 0.943;
                    self.channel1Max_ = 0.055;
                    self.channel2Min_ = 0.200;
                    self.channel2Max_ = 1.000;
                    self.channel3Min_ = 0.000;
                    self.channel3Max_ = 1.000;
                    self.SliderBW_ = ( (self.I_(:,:,1) >= self.channel1Min_) | (self.I_(:,:,1) <= self.channel1Max_) ) & ...
                        (self.I_(:,:,2) >= self.channel2Min_ ) & (self.I_(:,:,2) <= self.channel2Max_) & ...
                        (self.I_(:,:,3) >= self.channel3Min_ ) & (self.I_(:,:,3) <= self.channel3Max_);
                case 3
                    
                    self.channel1Min_ = 0.282;
                    self.channel1Max_ = 0.435;
                    self.channel2Min_ = 0.200;
                    self.channel2Max_ = 1.000;
                    self.channel3Min_ = 0.000;
                    self.channel3Max_ = 1.000;
                    self.SliderBW_ = ( (self.I_(:,:,1) >= self.channel1Min_) & (self.I_(:,:,1) <= self.channel1Max_) ) & ...
                        (self.I_(:,:,2) >= self.channel2Min_ ) & (self.I_(:,:,2) <= self.channel2Max_) & ...
                        (self.I_(:,:,3) >= self.channel3Min_ ) & (self.I_(:,:,3) <= self.channel3Max_);
                case 4
                    
                    self.channel1Min_ = 0.607;
                    self.channel1Max_ = 0.760;
                    self.channel2Min_ = 0.200;
                    self.channel2Max_ = 1.000;
                    self.channel3Min_ = 0.000;
                    self.channel3Max_ = 1.000;
                    self.SliderBW_ = ( (self.I_(:,:,1) >= self.channel1Min_) & (self.I_(:,:,1) <= self.channel1Max_) ) & ...
                        (self.I_(:,:,2) >= self.channel2Min_ ) & (self.I_(:,:,2) <= self.channel2Max_) & ...
                        (self.I_(:,:,3) >= self.channel3Min_ ) & (self.I_(:,:,3) <= self.channel3Max_);
                case 5
                    
                    self.channel1Min_ = 0.086;
                    self.channel1Max_ = 0.222;
                    self.channel2Min_ = 0.628;
                    self.channel2Max_ = 1.000;
                    self.channel3Min_ = 0.000;
                    self.channel3Max_ = 1.000;
                    self.SliderBW_ = ( (self.I_(:,:,1) >= self.channel1Min_) & (self.I_(:,:,1) <= self.channel1Max_) ) & ...
                        (self.I_(:,:,2) >= self.channel2Min_ ) & (self.I_(:,:,2) <= self.channel2Max_) & ...
                        (self.I_(:,:,3) >= self.channel3Min_ ) & (self.I_(:,:,3) <= self.channel3Max_);
            end
        end
        
        function SetRed(self)
            self.colour_ = 2;
            SetColour(self);
        end
        
        function SetGreen(self)
            self.colour_ = 3;
            SetColour(self);
        end
        
        function SetBlue(self)
            self.colour_ = 4;
            SetColour(self);
        end
        function SetYellow(self)
            self.colour_ = 5;
            SetColour(self);
        end
        
        function [X,Y] = DrawBoundaryAndRectangle(self)
            [B, L] = bwboundaries(self.SliderBW_,'noholes');
            hold on
            for k = 1:length(B)
                boundary = B{k};
                plot(boundary(:,2), boundary(:,1), 'cyan', 'LineWidth', 2)
            end
            hold on
            
            [X,Y] = DrawRect(L,1);
            
        end
        function [X,Y] = getImageCoordinate(self)
            [B, L] = bwboundaries(self.SliderBW_,'noholes');
            [X,Y] = DrawRect(L,2);
        end
    end
end