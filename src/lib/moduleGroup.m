classdef moduleGroup < handle
    %MODULEGROUP 此处显示有关此类的摘要
    %   此处显示详细说明
    
    properties
        ModuleList  (:, 1)  module        % An array of module pointers
        Boundary    (2, 2)  double        % Rectangular bound [xmin, ymin; xmax, ymax]
        Size        (1, 1)  int32         % Number of modules
%         LeadRobot   (1, 1)  robot         % leading robot of the module
%         DockGpIDs           int32         % The module group to dock with
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
                    all_loc(i, :) = module_lst(i).Location(1:2);
                end
                obj.Boundary(1, :) = [min(all_loc(:, 1)), min(all_loc(:, 2))];
                obj.Boundary(2, :) = [max(all_loc(:, 1)), max(all_loc(:, 2))];
            else
                obj.Size = 0;
                obj.Boundary = [0, 0; 0, 0];
            end
        end
        
        function bound = getBoundary(obj)
            all_loc = zeros(obj.Size, 2);
            for i=1:obj.Size
                all_loc(i, :) = obj.ModuleList(i).Location(1:2);
            end
            obj.Boundary(1, :) = [min(all_loc(:, 1)), min(all_loc(:, 2))];
            obj.Boundary(2, :) = [max(all_loc(:, 1)), max(all_loc(:, 2))];
            bound = obj.Boundary;
        end
        
        function ids = getIDs(obj)
            ids = zeros(1, obj.Size);
            for i=1:obj.Size
                ids(i) = obj.ModuleList(i).ID;
            end
        end
        
        function locs = getLocations(obj)
            locs = zeros(obj.Size, 3);
            for i=1:obj.Size
                locs(i, :) = obj.ModuleList(i).Location;
            end
        end
        
        function m = getModule(obj, id)
            % Input: a module ID
            % Output: the corresponding module object
            mlst = obj.ModuleList;
%             for temp_m=mlst
            for i=1:obj.Size
                temp_m = mlst(i);
                if temp_m.ID == id
                    m = temp_m;
                    return
                end
            end
            error("Get module object failed. Module "+string(id)+...
                "is not in the inquired module group.");
        end
        
        function next_id = getNextModuleID(obj, id)
            % Input: a module ID (or 0)
            % Output: return the module with the smallest id which larger
            % than the input id
            id_lst = obj.ModuleList.ID;
            if id == max(id_lst)
                % Rotate back to the robot.
                next_id = 0;
                return
            end
            temp_id = Inf;
            for cur_id = id_lst
                if cur_id > id && cur_id < temp_id
                    temp_id = cur_id;
                end
            end
            next_id = temp_id;
        end
        
        function AddModule(obj, module)
            obj.ModuleList = [obj.ModuleList; module];
            obj.Size = obj.Size + 1;
            % Update the boundary.
            if all(all(obj.Boundary == [0 0; 0 0]))
                obj.Boundary = [module.Location; module.Location];
            else
                obj.getBoundary();
                obj.Boundary(1, :) = min(obj.Boundary(1, :), target.Location);
                obj.Boundary(2, :) = max(obj.Boundary(2, :), target.Location);
            end
        end
        
        function AddModuleGp(obj, moduleGp)
            %METHOD1 此处显示有关此方法的摘要
            %   此处显示详细说明
            obj.ModuleList = [obj.ModuleList; moduleGp.ModuleList];
            obj.getBoundary();
            modGpB = moduleGp.getBoundary();
            obj.Size = obj.Size + moduleGp.Size;
            obj.Boundary(1, :) = min(obj.Boundary(1, :), modGpB(1, :));
            obj.Boundary(2, :) = max(obj.Boundary(2, :), modGpB(2, :));
        end
        
        function success = dock(obj, modules)
            % Active dock to other moduleGroup.
            if ~obj.checkNeighbour("module", modules)
                success = false; 
                return
            end
            % Change the allocation of modules in the 2 groups.
            obj.AddModuleGp(modules);
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
                bound = options.module.getBoundary();
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
            obj.getBoundary();
            newb(1, :) = min(bound(1, :), obj.Boundary(1, :));
            newb(2, :) = max(bound(2, :), obj.Boundary(2, :));
        end
        
        function a = getArea(obj, bound)
            if nargin == 1
                % calculate the area of itself.
                bound = obj.getBoundary();
            end
            a = (bound(2, 1) - bound(1, 1) + 1) * (bound(2, 2) - bound(1, 2) + 1);
        end
    end
end

