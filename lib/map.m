classdef map < handle
    %MAP 此处显示有关此类的摘要
    %   此处显示详细说明
    
    properties
        mapSize        (1, 2) int32    % Map Size: (x, y)
        obstacleMap           logical  % Obstacle map (1 means occupied)
        robotMap              logical  % Robot map
        moduleMap             logical  % Module map
        targetMap             logical  % Target map
        structureMap          logical  % Final target map
        robotDist             int32    % Cognition distance of robot
        moduleDist            int32    % Cognition distance of module
    end
    
    methods
        function obj = map(Nx, Ny)
            %MAP 构造此类的实例
            %   此处显示详细说明
            obj.mapSize = [Nx, Ny];
            obj.obstacleMap = zeros(Nx, Ny);
            obj.robotMap = zeros(Nx, Ny);
            obj.moduleMap = zeros(Nx, Ny);
            obj.targetMap = zeros(Nx, Ny);
            obj.structureMap = zeros(Nx, Ny);
        end
        
        function setDist(obj, options)
            arguments
                obj
                options.robot   double
                options.module  double
            end
            if isfield(options, "robot")
                obj.robotDist = options.robot;
            end
            if isfield(options, "module")
                obj.moduleDist = options.module;
            end
        end
        
        function local_map = getMap(obj, loc, type)
            % getMap
            %   此处显示详细说明
            if type == "r"
                dist = obj.robotDist;
            elseif type == "m"
                dist = obj.moduleDist;
            else
                error("Wrong input of element type: "+string(type)+". type must be r or m.");
            end
            local_map = zeros(obj.mapSize(1), obj.mapSize(2));
            l = max(loc(1)-dist, 1);               % left margin
            r = min(loc(1)+dist, obj.mapSize(1));  % right margin
            d = max(loc(2)-dist, 1);               % down (lower margin)
            u = min(loc(2)+dist, obj.mapSize(2));  % upper margin
%             disp("l: "+string(l));
%             disp("r: "+string(r));
%             disp("d: "+string(d));
%             disp("u: "+string(u));
            mo = obj.obstacleMap(l:r, d:u);        % map obstacle (local)
            mr = obj.robotMap(l:r, d:u);           % map robot
            mm = obj.moduleMap(l:r, d:u);          % map module
            ml = mo | mr | mm;                     % combined local map
            local_map(l:r, d:u) = ml;
        end
        
        function showMap(options)
            
        end
    end
end

