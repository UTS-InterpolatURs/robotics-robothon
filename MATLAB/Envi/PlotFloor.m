function [] = PlotFloor()
    surf([-3,-3;3,3],[-3,3;-3,3],[-0.91,-0.9;-0.91,-0.9],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
end