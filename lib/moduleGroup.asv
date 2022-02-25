classdef moduleGroup < handle
    %MODULEGROUP 此处显示有关此类的摘要
    %   此处显示详细说明
    
    properties
        ModuleList  (:, 1)  module        % An array of module pointers
        Boundary    (2, 2)  double        % Rectangular bound [xmin, ymin; xmax, ymax]
        Size        (1, 1)  int32         % Number of modules
        LeadRobot   (1, 1) robot   % leading robot of the module
    end
    
    methods
        function obj = moduleGroup(module_lst)
            %MODULEGROUP 构造此类的实例
            %   此处显示详细说明
            if nargin > 0
                obj.ModuleList = module_lst;
                obj.Size = length(module_lst);
                all_loc = zeros(obj.Size, 2);
                for i=1:obj.Size
                    all_loc(i, :) = module_lst(i).Location;
                end
                obj.Boundary(1, :) = [min(all_loc(:, 1)), min(all_loc(:, 2))];
                obj.Boundary(2, :) = [max(all_loc(:, 1)), max(all_loc(:, 2))];
            else
                obj.Size = 0;
                obj.Boundary = [0, 0; 0, 0];
            end
        end
        
        function AddModule(obj, module)
            obj.ModuleList = [obj.ModuleList; module];
            obj.Size = obj.Size + 1;
            % Update the boundary.
            if all(all(obj.Boundary == [0 0; 0 0]))
                obj.Boundary = [module.Location; module.Location];
            else
                obj.Boundary(1, :) = min(obj.Boundary(1, :), target.Location);
                obj.Boundary(2, :) = max(obj.Boundary(2, :), target.Location);
            end
        end
        
        function AddModuleGp(obj, moduleGp)
            %METHOD1 此处显示有关此方法的摘要
            %   此处显示详细说明
            obj.ModuleList = [obj.ModuleList; moduleGp.ModuleList];
            obj.Size = newobj.Size + targetGp.Size;
            obj.Boundary(1, :) = min(newobj.Boundary(1, :), targetGp.Boundary(1, :));
            obj.Boundary(2, :) = max(newobj.Boundary(2, :), targetGp.Boundary(2, :));
        end
        
        function success = dock(obj, options)
            % Active dock to other robot or moduleGroup.
            arguments
                obj
                options.robot  robot
                options.module moduleGroup
            end
%             if ~obj.checkNeighbour(options)
%                 success = false; 
%                 return
%             end
            if isfield(options, "robot")
                if ~obj.checkNeighbour(options.robot)
                    success = false; 
                    return
                end
                obj.LeadRobot = options.robot;
                obj.PosShift = options.robot.Location - obj.Location;
            elseif isfield(options, "module")
                if ~obj.checkNeighbour(options.module)
                    success = false; 
                    return
                end
                % Change the allocation of modules in the 2 groups.
                obj.AddModuleGp(options.module);
                % options.module.LeadRobot.carriedModule = moduleGroup();
                % TODO: 上一行要不要改成把options.module设置为空？
                options.module = moduleGroup();
                options.module.LeadRobot.isCarrying = false;
            else
                success = false;
                return
            end
            obj.Status = 1;
            success = true;
        end
        
        function decision = checkNeighbour(obj, options)
            % dock with a robot or a module group
            arguments
                obj
                options.robot  robot
                options.module moduleGroup
            end
            if isfield(options, "robot") && obj.Size == 1
                % TODO: only allow dock when the group has only 1 module.
                robloc = options.robot.Location;
                objloc = obj.ModuleList(1).Location;
                if all(abs(objloc - robloc) == [0, 1, 0]) || ...
                        all(abs(objloc - robloc) == [1, 0, 0])
                    decision = true;
                else
                    decision = false;
                end
            elseif isfield(options, "module")
                % check the boundary
                % TODO: change to check the face that suppose to be docked.
                bound = options.module.Boundary;
                self_a = obj.getArea();
                module_a = obj.getArea(bound);
                newbound = obj.combinedBound(bound);
                new_a = obj.getArea(newbound);
                if self_a + module_a == new_a
                    decision = true;
                else
                    decision = false;
                end
            else
                decision = false;
            end
        end
        
        function newb = combinedBound(obj, bound)
            newb = zeros(2, 2);
            newb(1, :) = min(bound(1, :), obj.Boundary(1, :));
            newb(2, :) = max(bound(2, :), obj.Boundary(2, :));
        end
        
        function a = getArea(obj, bound)
            if nargin == 1
                % calculate the area of itself.
                bound = obj.Boundary;
            end
            a = (bound(2, 1) - bound(1, 1) + 1) * (bound(2, 2) - bound(1, 2) + 1);
        end
    end
end

