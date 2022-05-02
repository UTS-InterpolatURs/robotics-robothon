classdef ImageStorage <handle
    properties
        image_array_ = []
    end
    methods
        function self = ImageStorage(rows,cols, image_num)
            self.image_array_ = zeros(rows,cols,image_num);
        end
        function addImage(self,image,index)
            self.image_array_(:,:,index) = image;     
        end
        function [r] = getImage(self,index)
            r = self.image_array_(:,:,index);
        end
    end
end