function [X,Y] = DrawRect(BW)
st = regionprops(BW, 'BoundingBox','Area','Centroid');
X = 0;
Y = 0;
try
count = 1;
process = st(1).Area; 
for k = 1 : length(st)
    if st(k).Area > process
        process = st(k).Area;
        count = k;
    end
end
thisCentroidBB = st(count).Centroid;
thisBB = st(count).BoundingBox;
rectangle('Position', [thisBB(1),thisBB(2),thisBB(3),thisBB(4)],...
    'EdgeColor','cyan','LineWidth',2 )
a=text(thisBB(1),thisBB(2)-30, strcat('X: ', num2str(round(thisCentroidBB(1))), ' Y: ', num2str(round(thisCentroidBB(2)))));
set(a, 'FontName', 'Arial', 'FontWeight', 'bold', 'FontSize', 12, 'Color', 'magenta');
% try delete(a); end
X = round(thisCentroidBB(1));
Y = round(thisCentroidBB(2));
end
end