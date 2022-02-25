classdef obstacle
    %OBSTACLE 此处显示有关此类的摘要
    %   此处显示详细说明
    
    properties
        Locations      (:, 2) double      % Obstacle locations [x, y]
        GlobalMap      map
        Size           (1, 1)  int32      % Number of obstacles
    end
    
    methods
        function obj = obstacle(locs, gmap)
            %OBSTACLE 构造此类的实例
            %   此处显示详细说明
            obj.Locations = locs;
            obj.GlobalMap = gmap;
            [obj.Size, ~] = size(locs);
            for i=1:obj.Size
                obj.GlobalMap.obstacleMap(obj.Locations(i, 1), obj.Locations(i, 2)) = 1;
            end
        end
        
        function add(obj, newloc)
            %METHOD1 此处显示有关此方法的摘要
            %   此处显示详细说明
            obj.Locations = [obj.Locations; newloc];
            obj.Size = obj.Size + 1;
            obj.GlobalMap.obstacleMap(newloc(1), newloc(2)) = 1;
        end
    end
end

