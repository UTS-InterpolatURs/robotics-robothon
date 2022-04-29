classdef ImageStorage
    properties
        image_array_ = []
    end
    methods
        function self = ImageStorage(image_size, image_num)
            self.image_array_ = zeros(image_size,image_size,image_num);
        end
        function addImage(self,image,index)
            targetSize = [length(image(:,1)) length(image(:,1))];
            win1 = centerCropWindow2d(size(image),targetSize);
            croppedImage = imcrop(image,win1);
            self.image_array_(:,:,index) = croppedImage;     
        end
        function [r] = getImage(self,index)
            r = self.image_array_(:,:,index);
        end
    end
end