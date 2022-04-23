function SimulateEnvironment(inputEnvironment)
    surf([-5,-5;5,5],[-5,5;-5,5],[-0.7,-0.7;-0.7,-0.7],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
    hold on;
    % table
    
    [f,v,data] = plyread(inputEnvironment,'tri');
    % Get vertex count
    tableVertexCount = size(v,1);
    
    % Move center point to origin
    midPoint = sum(v)/tableVertexCount;
    tableVerts = v - repmat(midPoint,tableVertexCount,1);
    
    % Create a transform to describe the location (at the origin, since it's centered
    tablePose = eye(4);
    
    vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
    
    hold on;
    tableMesh_h = trisurf(f,v(:,1),v(:,2), v(:,3) ...
        ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
    
    tablePose = tablePose*transl(0,0,-0.2);
    updatedPoints = [tablePose * [tableVerts,ones(tableVertexCount,1)]']';
    
    % Now update the Vertices
    tableMesh_h.Vertices = updatedPoints(:,1:3);
    drawnow();
    
    % Turn on a light (only turn on 1, don't keep turning them on), and make axis equal
    camlight;
    axis equal;
    hold on;
end