classdef Episode < handle
    %EPISODE 此处显示有关此类的摘要
    %   此处显示详细说明
    
    properties
        trial                Trial
        alg_type             int32
        runSimExp      (1,2) logical
    end
    
    methods
        function obj = Episode(runSimExp, type)
            %EPISODE 构造此类的实例
            %   此处显示详细说明
            if runSimExp == "S" || runSimExp == "s"
                obj.runSimExp = [true, false];
            elseif runSimExp == "E" || runSimExp == "e"
                obj.runSimExp = [false, true];
            elseif runSimExp == "SE" || runSimExp == "se"
                obj.runSimExp = [true, true];
            end
            obj.alg_type = type;
        end
        
        function runExp(obj)
            %METHOD1 此处显示有关此方法的摘要
            %   此处显示详细说明
            % General properties
            Map_Size = [16, 16]; % length*width
            rob_loc = [1, 16; 16, 16; 1, 1; 16, 1];
            shape_list = ["S", "T", "L", "1", "q"];
            N = length(shape_list);
            for i=1:N
                t_name = shape_list(i)+"-4t-4r-"+string(obj.alg_type);
                obj.trial(i) = Trial(t_name);
                % Map
                obj.trial(i).gmap = map(Map_Size, 3);
                % set robot
                d = obj.getDocks(shape_list(i));
                obj.trial(i).setRobots(rob_loc, d);
                % Set logging if log is required
                obj.trial(i).islogging = [true, true];
                obj.trial(i).setLogging(true);
                t = obj.getTargets(shape_list(i));
                [success, rotated_dock] = obj.trial(i).setTargets(obj.alg_type,...
                     t, t(2,:));
                if ~success
                    disp("Program exit ...")
                else
                    obj.trial(i).setDisplay(rotated_dock, 6);
                    % Main loop
                    max_steps = 1e6;
                    while ~obj.trial(i).structure_arrive && ...
                            obj.trial(i).step_cnt <= max_steps
                        obj.trial(i).execute();
                    end
                end
                obj.trial(i).endSim;
            end
        end
        
        function t = getTargets(~, shape)
            switch shape
                case "S"   % S-shape
                    t = [4, 4; 5, 4; 5, 5; 6, 5];
                case "T"   % T-shape
                    t = [4, 5; 5, 5; 6, 5; 5, 4];
                case "L"   % L-shape
                    t = [4, 4; 4, 5; 5, 5; 6, 5];
                case "1"   % stick (one) - shape
                    t = [4, 4; 5, 4; 6, 4; 7, 4];
                case "q"   % squared shape
                    t = [4, 5; 5, 5; 4, 4; 5, 4];
            end
            t = t + 2;
        end

        function d = getDocks(obj, shape)
            type = obj.alg_type;
            shape_list = ["S", "T", "L", "1", "q"];
            N = 5;
        %     shape_list = ["S", "L", "1", "q"];
            dock = cell(1, N);
            if type <= 2
                dock(1) = {[0, 1, 0, 1; 0, 0, 1, 0; 0, 0, 0, 1; 1, 0, 1, 0]};
                dock(2) = {[0, 1, 1, 1; 0, 0, 1, 0; 0, 0, 0, 1; 1, 0, 0, 0]};
                dock(3) = {[0, 0, 1, 1; 0, 0, 1, 0; 0, 1, 0, 1; 1, 0, 0, 0]};
                dock(4) = {[0, 0, 1, 1; 0, 0, 1, 0; 0, 0, 0, 1; 0, 0, 1, 1]};
                dock(5) = {[0, 1, 0, 1; 1, 0, 0, 1; 0, 0, 1, 0; 0, 0, 1, 0]};
            end

            if type == 3
                % assign 2 types of dock joints
                dock(1) = {[0, 1, 0, 1; 0, 0, 2, 0; 0, 0, 0, 1; 2, 0, 2, 0]};
                dock(2) = {[0, 1, 2, 1; 0, 0, 2, 0; 0, 0, 0, 1; 2, 0, 0, 0]};
                dock(3) = {[0, 0, 1, 1; 0, 0, 2, 0; 0, 2, 0, 2; 1, 0, 0, 0]};
                dock(4) = {[0, 0, 2, 1; 0, 0, 2, 0; 0, 0, 0, 1; 0, 0, 2, 1]};
                dock(5) = {[0, 1, 0, 2; 2, 0, 0, 2; 0, 0, 1, 0; 0, 0, 1, 0]};
        %         for i=1:N
        %             [r, c] = find(dock{i});
        %             p = randperm(length(r), length(r)/2);
        %             for j=1:length(p)
        %                 dock{i}(r(p(j)), c(p(j))) = 2;
        %             end
        %         end
            end
            if type >= 2
                % random shuttle
                dock = obj.rand_rotate(dock);
        %         for i = 1:N
        %             for j = 1:N
        %                 dock{i} = DoRotation(dock{i}, j
            end
            for i = 1:N
                if shape == shape_list(i)
                    d = dock{i};
                    return
                end
            end
            error("The input shape is "+shape+", which is not in the shape list.");
        end

        function new_dock = rand_rotate(obj, cell_dock)
            N_exp = length(cell_dock);
            [N_rob, Nd] = size(cell_dock{1});
            new_dock = cell(1, N_exp);
            for i=1:N_exp
                new_dock{i} = zeros(N_rob, Nd);
                for j=1:N_rob
                    new_dock{i}(j,:) = obj.rotate(cell_dock{i}(j,:), randi([0,3],1));
                end
            end
        end

        function new_d = rotate(~, d, num)
            switch num
                case 0
                    new_d = d;
                case 1
                    new_d = d([3 4 2 1]);
                case 2
                    new_d = d([2 1 4 3]);
                case 3
                    new_d = d([4 3 1 2]);
            end
        end

    end
end

