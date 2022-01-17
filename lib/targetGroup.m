classdef targetGroup < matlab.mixin.Copyable
    % Group of all targets
    properties (Access = public)
        TargetList  (:, 1)  targetPoint   % An array of target pointers
        Boundary    (2, 2)  double        % Rectangular bound [xmin, ymin; xmax, ymax]
        Size        (1, 1)  int32         % Number of targets
    end
    
    methods (Access = public)
        function obj = targetGroup(target_lst)
            if nargin > 0
                obj.TargetList = target_lst;
                obj.Size = length(target_lst);
                all_loc = zeros(obj.Size, 2);
                for i=1:obj.Size
                    all_loc(i, :) = target_lst(i).Location;
                end
                % all_loc = [target_lst.Location.'].';
%                 disp(all_loc);
                obj.Boundary(1, :) = [min(all_loc(:, 1)), min(all_loc(:, 2))];
                obj.Boundary(2, :) = [max(all_loc(:, 1)), max(all_loc(:, 2))];
            else
%                 disp('here');
%                 obj.TargetList = [];
                obj.Size = 0;
                obj.Boundary = [0, 0; 0, 0];
            end
        end
        
        function obj = AddTarget(obj, target)
            % Add to the target list.
            obj.TargetList = [obj.TargetList; target];
            obj.Size = obj.Size + 1;
            % Update the boundary.
            if all(all(obj.Boundary == [0 0; 0 0]))
                obj.Boundary = [target.Location; target.Location];
            else
                obj.Boundary(1, :) = min(obj.Boundary(1, :), target.Location);
                obj.Boundary(2, :) = max(obj.Boundary(2, :), target.Location);
            end
        end
        
        function newobj = AddTargetGp(obj, targetGp)
            % Add a group of target to the current target group.
            newobj = copy(obj);
            newobj.TargetList = [newobj.TargetList; targetGp.TargetList];
            newobj.Size = newobj.Size + targetGp.Size;
            newobj.Boundary(1, :) = min(newobj.Boundary(1, :), targetGp.Boundary(1, :));
            newobj.Boundary(2, :) = max(newobj.Boundary(2, :), targetGp.Boundary(2, :));
        end
        
        function [tar_left, tar_right] = TargetSplitting(obj, option, c, exl)
%             j = 1;
            temp_mul = zeros(obj.Size, 1);
%             disp("nargin: "); disp(nargin);
            if nargin < 4
                exl = 1;
            end
            for i = (obj.Boundary(1, option+1)+1):obj.Boundary(2, option+1)
                [temp_l, temp_r] = obj.GetSplitNum(i, option);
%                 disp("left");
%                 disp(temp_l);
%                 disp("right");
%                 disp(temp_r);
                temp_mul(i) = temp_l * temp_r;
%                 disp("multiple");
%                 disp(temp_mul);
%                 j = j + 1;
            end
            [~, i] = max(temp_mul);
%             disp("max index");
%             disp(i);
            [tar_left, tar_right] = obj.GetSplitGroup(i, option, c, exl);
        end
        
        function result = CanBeSplit(obj)
            if obj.Boundary(2, 1) - obj.Boundary(1, 1) >= 1
                result(1) = true;
            else
                result(1) = false;
            end
            if obj.Boundary(2, 2) - obj.Boundary(1, 2) >= 1
                result(2) = true;
            else
                result(2) = false;
            end
        end
    end
    
    methods (Access = private)
        function [nl, nr] = GetSplitNum(obj, pos, option)
            nl = 0;  % Left or lower part
            nr = 0;  % Right or upper part
%             if option
%                 % Cut vertically
            for i = 1:obj.Size
                if obj.TargetList(i).Location(option+1) < pos
                    nl = nl + 1;
                else
                    nr = nr + 1;
                end
            end
%             else
%                 % Cut horizontally
%                 
%             end
        end
        
        function [tar_l, tar_r] = GetSplitGroup(obj, pos, option, c, exl)
            % pos: splitting line (x = pos or y = pos)
            % option = 0 (vertical) or = 1 (horizontal)
            % c: expansion center [x, y]
            % exl: extension length (= 1 by default)
            if c(1) < obj.Boundary(1, 1) || c(2) < obj.Boundary(1, 2)
                c(1) = max(c(1), obj.Boundary(1, 1));
                c(2) = max(c(2), obj.Boundary(1, 2));
            end
            if c(1) > obj.Boundary(2, 1) || c(2) > obj.Boundary(2, 2)
                c(1) = min(c(1), obj.Boundary(2, 1));
                c(2) = min(c(2), obj.Boundary(2, 2));
            end
            if nargin < 5
                exl = 1;
            end
            tar_l = targetGroup();  % Left or lower targets
            tar_r = targetGroup();  % Right or upper targets
            for i = 1:obj.Size
                tar = copy(obj.TargetList(i));
                if tar.Location(option+1) < pos
                    if c(option+1) >= pos
                        e_delta = c(option+1) - pos + 1;
                        tar.Location(option+1) = tar.Location(option+1) - exl*e_delta;
                    end
                    tar_l.AddTarget(tar);
                else
                    if c(option+1)< pos
                        e_delta = pos - c(option+1);
                        tar.Location(option+1) = tar.Location(option+1) + exl*e_delta;
                    end
                    tar_r.AddTarget(tar);
                end
            end
            
        end
        
    end
end
