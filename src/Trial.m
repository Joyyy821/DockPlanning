classdef Trial < handle
    %TRIAL 此处显示有关此类的摘要
    %   此处显示详细说明
    
    properties
        % Classes
        gmap               map
        tars               targetGroup
        ext                Extension
        robotGp            AssembleGroup
        floatGp            moduleGroup
        mods               module      % Only save the floating modules
        obstacles          obstacle
        display            display2D
        % Flags
        fetch_arrive       logical
        target_arrive      logical
        structure_arrive   logical
        finish             logical
        % params
        N_rob              int32
        N_mod              int32
        rob_dirs           int32
        % TODO: 不应该存robdir 应该在extension
        % class导入之后就把哪个编号的模块可以对接哪些面告诉所有机器人group
        ci
    end
    
    methods
        function obj = Trial()
            %TRIAL 构造此类的实例
            %   此处显示详细说明
            addpath('lib');
            addpath('lib/display'); addpath('lib/alg');
            
        end
        
        function execute(obj)
            %METHOD1 此处显示有关此方法的摘要
            %   此处显示详细说明
                        
            % Walk the modules
            for i=1:length(obj.mods)
                obj.mods(i).walk();
            end
            % TODO: move the floating module groups
            
            % Move the robot groups (search or construct)
            for i=1:obj.N_rob
                obj.ci = i;
                if ~obj.fetch_arrive(i)
                    % 关于机器人搜索模块的部分，主体由调用assemblegroup方法
                    % 完成，该方法使机器人在地图内S型搜索（设定行或列的终点为
                    % goal，每次move一步（A*）并访问地图（访问时地图返回浮动模块信息
                    % 即存在于module map但不在group map的模块），一旦发现浮动模块，立即把
                    % goal设置为模块当前位置（的旁边，具体方向需要由模块id对应
                    % 的target id决定，以避免遮挡对接面，并设置机器人自己的
                    % ignore pos，在之后追踪模块的过程中每次move前更新。所以
                    % 这一步需要访问extension，如何完成？），并在之后每一步
                    % 移动前更新模块位置 并重新规划。如果模块离开视野，则从
                    % 当前位置开始继续S型搜索；如果成功到达模块边上，则
                    % assemblegroup的方法需返回模块id，在此方法中通过模块id
                    % 二分搜索模块index，完成当前robotGp与模块对接，完成fetch
                    
                    % 关于更新ignore pos的部分，robot updatemap函数的设置
                    % 可以改成：ignore pos只是不对pos进行扩张，而不是直接把
                    % 这个pos设为空
                    id_m = obj.robotGp(i).search();
                    if ~isempty(id_m)
                        [float_m, m_idx] = obj.getModule(id_m);
                        obj.robotGp(i).updateGroup("add", float_m);
                        obj.mods(m_idx) = [];
                        obj.fetch_arrive(i) = true;
                        target = int32(evalin('base', 'Object_Current_Target'));
                        obj.robotGp(i).LeadRobot.Goal = ...
                            [target(id_m, :)+obj.robotGp(i).attachdir, 0];
                    end
                elseif ~obj.target_arrive(i)
                    obj.target_arrive(i) = obj.robotGp(i).move();
                elseif ~obj.structure_arrive(i)
                    
                elseif ~obj.finish(i)
                    
                else
                    disp("Construction task finished!");
                end
            end

            
            % update the display
            obj.display.updateMap("Robot", obj.getRobots(), ...
                "Module", obj.getAllMods());
        end
        
        function robs = getRobots(obj)
            robs = robot();
            for i=1:obj.N_rob
                robs(i) = obj.robotGp(i).LeadRobot;
            end
        end
        
        function setRobotGp(obj, locs)
            [l, ~] = size(locs);
            
            for i = 1:l
                rob = robot(i, [locs(i, :), 0], obj.gmap);
                obj.robotGp(i) = AssembleGroup(i, rob, obj.gmap);
                if ~isempty(obj.ext)
                    cl = length(obj.ext.GroupLayers);
                    obj.robotGp(i).cl = cl;
                end
                obj.robotGp(i).initSearch();
            end
            obj.fetch_arrive = false(l);
            obj.target_arrive = false(l);
            obj.structure_arrive = false(l);
            obj.finish = false(l);
            obj.rob_dirs = zeros(l, 2);
            obj.N_rob = l;
        end
        
        function [m, idx] = getModule(obj, id, l, r)
            if nargin == 2
                l = 1; r = length(obj.mods);
            end
            mid = int32((r-l)/2+l);
            if obj.mods(mid).ID == id
                m = obj.mods(mid);
                idx = mid;
                return
            elseif r-l == 1
                if obj.mods(r).ID == id
                    idx = r;
                elseif obj.mods(l).ID == id
                    idx = l;
                end
                m = obj.mods(idx);
                return
            elseif obj.mods(mid).ID > id
                r = mid;
            else
                l = mid;
            end
            [m, idx] = obj.getModule(id, l, r);
        end
        
        function ms = getAllMods(obj)
            ms = obj.mods;
            for i=1:obj.N_rob
                if ~isempty(obj.robotGp(i).attachModuleID)
                    ms = [ms, obj.robotGp(i).modules.ModuleList.'];
                end
            end
            for i=1:length(obj.floatGp)
                ms = [ms, obj.floatGp.ModuleList.'];
            end
        end
        
        function setModules(obj, locs)
            [obj.N_mod, ~] = size(locs);
            for i = 1:obj.N_mod
                obj.mods(i) = module(i, [locs(i, :), 0], obj.gmap);
            end
%             obj.N_mod = length(modules);
        end
        
        function setTargets(obj, w, l, c, ext_c, ext_l)
            % Input: width, length, coordinate of lower left corner,
            % extension center, extension length
            N = w*l;
            Tars = [];
            for i = 1:N
                m = ceil(i/l);
                n = i - (m-1) * l;
                Tars = [Tars; targetPoint(i, [m+c(1)-1, n+c(2)-1], obj.gmap)];
            end
            obj.tars = targetGroup(Tars);
            
            % Binary tree construction / extension
            obj.ext = Extension(obj.tars, obj.gmap);
            obj.ext.TargetToTree(ext_c, ext_l);
        end
        
        function setDisplay(obj)
            % Display
            robs = obj.getRobots();
            obj.display = display2D(obj.gmap.mapSize, ...
                                    "FinalTarget", obj.tars, ...
                                    "Robot", robs, ...
                                    "Module", obj.mods, ...
                                    "Obstacle", obj.obstacles.Locations);
            % Extension
            obj.ext.showExtension(obj.display);
        end
    end
end

