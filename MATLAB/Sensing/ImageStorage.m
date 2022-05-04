classdef ImageStorage <handle
    properties
        depth_image_array_ = [];
        rgb_image_array_ = [];
    end
    methods
        function self = ImageStorage(rows,cols, image_num)
            self.depth_image_array_ = zeros(rows,cols,image_num);
            self.rgb_image_array_ = zeros(rows,cols,3,image_num);
        end
        function addDepthImage(self,image,index)
            self.depth_image_array_(:,:,index) = image;
        end
        function addRGBImage(self,image,index)
            for i = 1 : 3
                self.rgb_image_array_(:,:,i,index) = image(:,:,i);
            end
        end
        function [r] = getDepthImage(self,index)
            r = self.depth_image_array_(:,:,index);
        end
        function [rgb] = getRGBImage(self,index)
            for i = 1:3
                rgb(:,:,i) = uint8(self.rgb_image_array_(:,:,i,index));
            end
        end
    end
end