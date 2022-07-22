classdef map < handle
    %MAP 此处显示有关此类的摘要
    %   此处显示详细说明
    
    properties
        mapSize        (1, 2) int32    % Map Size: (x, y)
        robotMap              int32    % Robot map
        workerRobotMap        int32    % Robot (with its carrying modules) which current status is working
        groupMap              int32    % Group map
        targetMap             int32    % Target map
        structureMap          logical  % Final target map
        robotDist = 3                  % Cognition distance of robot
        robotCogn             logical  % Robot cognition area (circle)
    end
    
    methods
        function obj = map(map_dim, rob_dist)
            %MAP 构造此类的实例
            %   此处显示详细说明
            if nargin == 0
                Nx = 0; Ny = 0;
            end
            if nargin >= 1
                Ny = map_dim(2);
                Nx = map_dim(1);
            end
            if nargin >= 2
                obj.robotDist = rob_dist;
            end
            obj.mapSize = [Nx, Ny];
            obj.robotMap = zeros(Nx, Ny);
            obj.workerRobotMap = zeros(Nx, Ny);
            obj.targetMap = zeros(Nx, Ny);
            obj.structureMap = zeros(Nx, Ny);
            obj.groupMap = zeros(Nx, Ny);
            obj.robotCogn = obj.getCircle(obj.robotDist);
        end
        
        function setDist(obj, rob_dist)
            obj.robotDist = rob_dist;
            obj.robotCogn = obj.getCircle(rob_dist);
        end
        
        function setStructureMap(obj, locs)
            [n, ~] = size(locs);
            for i=1:n
                obj.structureMap(locs(i,1), locs(i,2)) = 1;
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
        
        function [local_map, pr] = getMap(obj, loc)
            % getMap
            %   此处显示详细说明
            % pr = [];  % robot location + id (priority)
            dist = obj.robotDist;
            circle = obj.robotCogn;

            local_map = zeros(obj.mapSize(1), obj.mapSize(2));
            l = max(loc(2)-dist, 1);               % left margin
            r = min(loc(2)+dist, obj.mapSize(2));  % right margin
            d = max(loc(1)-dist, 1);               % down (lower margin)
            u = min(loc(1)+dist, obj.mapSize(1));  % upper margin
%             disp("l: "+string(l));
%             disp("r: "+string(r));
%             disp("d: "+string(d));
%             disp("u: "+string(u));
            mr = obj.robotMap(d:u, l:r);           % map robot
            mw = obj.workerRobotMap(d:u, l:r);     % map worker
            mg = obj.groupMap(d:u, l:r);           % map group
            ml = mr;                % combined local map (to a logical map)
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
                ri = find(rv==wks(i));
                pr(ri, 3) = 0;
            end
        end
        
        function g_ids = getGroupAroundTarget(obj, tar)
            t_bound = tar.Boundary;
            neighbour = [t_bound(1,1:2)-2; t_bound(2,1:2)+2];
            neighbour(1, 1:2) = max(neighbour(1, 1:2), 0);
            neighbour(2, 1) = min(neighbour(2, 1), obj.mapSize(1));
            neighbour(2, 2) = min(neighbour(2, 2), obj.mapSize(2));
            x_min = neighbour(1,1);x_max = neighbour(2,1);
            y_min = neighbour(1,2);y_max = neighbour(2,2);
%             local_map = xor(obj.groupMap, obj.workerRobotMap);
            local_map = obj.groupMap(x_min:x_max, y_min:y_max);
            [~, ~, g_ids] = find(local_map);
            g_ids = unique(g_ids);
        end
        
        function showMap(obj, optional_map)
            if nargin == 1
                disp("Robot map: ");
                obj.showMap(obj.robotMap);
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

