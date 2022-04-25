classdef map < handle
    %MAP 此处显示有关此类的摘要
    %   此处显示详细说明
    
    properties
        mapSize        (1, 2) int32    % Map Size: (x, y)
        obstacleMap           logical  % Obstacle map (1 means occupied)
        dockerMap             logical  % Docker map
        robotMap              int32    % Robot map
        workerRobotMap        int32    % Robot which current status is working
        moduleMap             int32    % Module map
        groupMap              int32    % Group map
        targetMap             int32    % Target map
        structureMap          logical  % Final target map
        robotDist = 3                  % Cognition distance of robot
        moduleDist = 3                 % Cognition distance of module
        robotCogn             logical  % Robot cognition area (circle)
        moduleCogn            logical  % Module cognition area (circle)
    end
    
    methods
        function obj = map(Nx, Ny)
            %MAP 构造此类的实例
            %   此处显示详细说明
            if nargin == 0
                Nx = 0; Ny = 0;
            end
            if nargin == 1
                Ny = Nx(2);
                Nx = Nx(1);
            end
            obj.mapSize = [Nx, Ny];
            obj.obstacleMap = zeros(Nx, Ny);
            obj.dockerMap = zeros(Nx, Ny);
            obj.robotMap = zeros(Nx, Ny);
            obj.workerRobotMap = zeros(Nx, Ny);
            obj.moduleMap = zeros(Nx, Ny);
            obj.targetMap = zeros(Nx, Ny);
            obj.structureMap = zeros(Nx, Ny);
            obj.groupMap = zeros(Nx, Ny);
            obj.robotCogn = obj.getCircle(obj.robotDist);
            obj.moduleCogn = obj.getCircle(obj.moduleDist);
        end
        
        function setDist(obj, options)
            arguments
                obj
                options.robot   int32
                options.module  int32
            end
            if isfield(options, "robot")
                obj.robotDist = options.robot;
                obj.robotCogn = obj.getCircle(options.robot);
            end
            if isfield(options, "module")
                obj.moduleDist = options.module;
                obj.moduleCogn = obj.getCircle(options.module);
            end
            
        end
        
        function cir = getCircle(obj, dist)
            % generate cogn area
            N = 2*dist+1;
            cir = ones(N, N);
            center = [dist+1, dist+1];
            for i=1:N
                for j=1:N
                    temp = obj.getLength([i, j], center);
%                     disp(temp);
                    if temp > dist
                        cir(i, j) = 0;
                    end
                end
            end
        end
        
        function len = getLength(~, x, y)
            len = sqrt(sum((x - y).^2));
        end
        
        function [local_map, m_loc, pr] = getMap(obj, loc, type)
            % getMap
            %   此处显示详细说明
            m_loc = [];
            pr = [];  % robot location + id (priority)
            if type == "r" || type == "search"
                dist = obj.robotDist;
                circle = obj.robotCogn;
            elseif type == "m"
                dist = obj.moduleDist;
                circle = obj.moduleCogn;
            else
                error("Wrong input of element type: "+string(type)+". type must be r or m.");
            end
            local_map = zeros(obj.mapSize(1), obj.mapSize(2));
            l = max(loc(2)-dist, 1);               % left margin
            r = min(loc(2)+dist, obj.mapSize(2));  % right margin
            d = max(loc(1)-dist, 1);               % down (lower margin)
            u = min(loc(1)+dist, obj.mapSize(1));  % upper margin
%             disp("l: "+string(l));
%             disp("r: "+string(r));
%             disp("d: "+string(d));
%             disp("u: "+string(u));
            mo = obj.obstacleMap(d:u, l:r);        % map obstacle (local)
            md = obj.dockerMap(d:u, l:r);          % map docker
            mr = obj.robotMap(d:u, l:r);           % map robot
            mw = obj.workerRobotMap(d:u, l:r);     % map worker
            mg = obj.groupMap(d:u, l:r);           % map group
            mm = obj.moduleMap(d:u, l:r);          % map module
            ml = mo | mr | mm | md;                % combined local map (to a logical map)
            % Combine local map with cognition circle
            cl = 1; cr = 2*dist+1; cu = 2*dist+1; cd = 1;
            if r-l == 2*dist && u-d == 2*dist
                ml = ml & circle;
            else
                
                if l == 1   % left equals to one
                    cl = 2*dist+1-(r-l);
                end
                if r == obj.mapSize(2)
                    cr = r-l+1;
                end
                if d == 1
                    cd = 2*dist+1-(u-d);
                end
                if u == obj.mapSize(1)
                    cu = u-d+1;
                end
%                 disp("cl: "+string(cl));
%                 disp("cr: "+string(cr));
%                 disp("cd: "+string(cd));
%                 disp("cu: "+string(cu));

                ml = ml & circle(cd:cu, cl:cr);
            end
            local_map(d:u, l:r) = ml;
            if type == "search"
                [r, c] = find(local_map);
                N = length(r);
                for i=1:N
                    if obj.moduleMap(r(i), c(i)) && ...
                            (~obj.groupMap(r(i), c(i)))
                        temp = [r(i), c(i)];
                        temp(3) = obj.moduleMap(r(i), c(i));
                        m_loc = [m_loc; temp];
%                         return
                    end
                end
            else  % type == r or type == m
                % TOODO: pr 从mr find出来 并和|circle的index进行比较，如果在
                % circle内则添加至pr
                % TODO: map 增加一个一个属性记录正在等待docking的机器人
                mr_del = mr & (~circle(cd:cu, cl:cr));
                [r_del, c_del] = find(mr_del);
                for i=1:length(r_del)
                    mg(r_del(i), c_del(i)) = 0;
                    mw(r_del(i), c_del(i)) = 0;
                end
                [rr, rc, rv] = find(mg);
                rr = rr + d - 1;
                rc = rc + l - 1;
                pr = [rr, rc, rv];
                [~, ~, wks] = find(mw);
                for i=1:length(wks)
                    ri = find(rv==wks(i), 1);
                    pr(ri, 3) = 0;
                end
                
                if ~isempty(pr)
                    disp("find groups:");
                    disp(pr);
                end
            end
        end
        
        function showMap(obj, optional_map)
            if nargin == 1
                disp("Obstacle map: ");
                obj.showMap(obj.obstacleMap);
                disp("Robot map: ");
                obj.showMap(obj.robotMap);
                disp("Module map: ");
                obj.showMap(obj.moduleMap);
                disp("Current taget map: ");
                obj.showMap(obj.targetMap);
                disp("Final target map: ");
                obj.showMap(obj.structureMap);
            else
                % show the input map
                map2show = optional_map.';
                disp(flip(map2show));
%                 figure;
%                 imagesc(map2show);
            end
        end
    end
end

