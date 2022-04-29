classdef PointCloud
    properties
        fdx_ = 0;
        fdy_ = 0;
        u0_ = 0;
        v0_ = 0;
        h_ = 0;
        w_ = 0;
        x_ = 0;
        y_ = 0;
    end
    methods
        function self = PointCloud(self)
        end
        function setExtrinsic(self,fdx,fdy,u0,v0,h,w)
            self.fdx_ = fdx;
            self.fdy_ = fdy;
            self.u0_ = u0;
            self.v0_ = v0;
            self.h_ = h;
            self.w_ = w;
        end
        
        function [pt] = getPointCloud(self,depthImage)
            u = repmat(1:self.w_,[self.h_,1]);
            v = repmat(1:self.h_,[self.w_,1])';
            Z = depthImage;
            Z = double(Z(:));
            X = (double(Z(:)).*(u(:)-self.u0_))/self.fdx_;
            Y = (double(Z(:)).*(v(:)-self.v0_))/self.fdy_;
            pt = [X(:),Y(:),Z(:)];
        end
    end
end