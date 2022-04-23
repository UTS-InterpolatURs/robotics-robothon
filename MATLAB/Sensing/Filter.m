function [filteredImage] = Filter(bw)
    filteredImage = bw;
    for i = 1:length(filteredImage)
        if length(filteredImage{i,1}) <= 5
                try delete(filteredImage{i,1}); end
        end
    end
end