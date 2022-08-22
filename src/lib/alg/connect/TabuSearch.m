classdef TabuSearch < handle 
    %TABUSEARCH Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % 
        point    % target positions
        dock     % dock status
        rob_loc  % robot initial locations
        % parameters
        N_tar
        N_rob
        isDockIden
        isRotation % rotation command
        % search params & statistics
        N_it        % iteration times for finding one solution
        solutions   % Feasible solutions
        opt_sol     % optimal solution
        ExitCode
        BestSolCode
        % Check the disp code methods at the bottom of this class
        % definition about the meaning for the ExitCode and BestSolCode
        % attributes.
        
%         CandLen = 1
    end
    
    methods
        function obj = TabuSearch(init_point,init_dock, init_loc, type)
            %TABUSEARCH Construct an instance of this class
            %   type = 1: w/o rotation
            %   type = 2: with rotation
            %   type = 3: A/B-type dock joint with rotation
            if nargin == 0
                obj.point = [4,4;5,4;5,5;6,5]; % x, y
                obj.dock = [1,0,0,0;...
                            2,0,1,0;...
                            0,1,0,0;...
                            1,0,2,0;...
                            1,2,0,0
                            ]; % up, down, left, right
                obj.rob_loc = [1, 1; 4, 1; 7, 1; 10, 1; 13, 1];
                obj.isDockIden = false;
                obj.isRotation = true;
%                 obj.dock = [0,0,0,1;...
%                             0,1,0,1;...
%                             0,0,1,0;...
%                             1,0,1,0]; % up, down, left, right
            elseif nargin >= 3
                obj.point = init_point;
                obj.dock = init_dock;
                obj.rob_loc = init_loc;
            end
            if nargin == 4
                switch type
                    case 1
                        obj.isRotation = false;
                        obj.isDockIden = true;
                    case 2
                        obj.isRotation = true;
                        obj.isDockIden = true;
                    case 3
                        obj.isRotation = true;
                        obj.isDockIden = false;
                end
%                 obj.isDockIden = is_dock_ident;
%             else
%                 obj.isDockIden = isempty(find(obj.dock==2,1));
            end
            [obj.N_tar, ~] = size(obj.point);
            [obj.N_rob, ~] = size(obj.dock);
%             obj.rotation = zeros(obj.N_rob, 1);
        end

        function setTS(obj, pt, d)
            obj.point = pt;
            obj.dock = d;
            [obj.N_tar, ~] = size(obj.point);
            [obj.N_rob, ~] = size(obj.dock);
        end
        
        function [sol, dock, cost] = search(obj)
%             obj.rotation = zeros(obj.N_rob, 1);
            obj.solutions = [];
            %% Input Validation
            % check if any robot has no dock joint
            N_invalid = 0;
            for i=1:obj.N_rob
                if ~any(obj.dock(i,:))
                    N_invalid = N_invalid + 1;
                end
            end
            if obj.N_rob - N_invalid < obj.N_tar
                error("Invalid input with "+string(N_invalid)+...
                    " out of "+string(obj.N_rob)+" robots having no "+...
                    "avaliable dock sites, but "+string(obj.N_tar)+...
                    " targets are given to connect.");
            end
            % check if the dock joints are not indetical but all robots
            % have only one kind of dock joint (all 1 or all 2).
            if ~obj.isDockIden
                if isempty(find(obj.dock==2,1)) && isempty(find(obj.dock==1,1))
                    error("Invalid input with all robots having only one "+...
                        "kind of dock joint (all 1 or all 2), which cannot"+...
                        " be connected.");
                end
            end
            %% Problem Definition
            
            CostFunction = @(p) obj.MyCost(p);    % Cost Function
            
%             [nQueen, ~] = size(obj.dock);   % Number of Robots
            
            ActionList = CreatePermActionList(obj.dock, obj.isRotation);    % Action List
            
            nAction = numel(ActionList);              % Number of Actions
            
            
            %% Tabu Search Parameters
            
            MaxIt = 1e5;                      % Maximum Number of Iterations
            
            TL = round(0.5*nAction);      % Tabu Length
            
%             CandLen = 4;               % Maximum number of feasible solutions to be recorded
            
            obj.N_it(1) = 1e3;
            
            SearchMulti = 10;           % Allowed multiple iteration times for next solution

            obj.ExitCode = 0;             % Indicate why the program stops
            
            obj.BestSolCode = -1;

            %% Initialization
            
            % Create Empty Individual Structure
            empty_individual.Position = [];
            empty_individual.Cost = [];
            empty_individual.Dock = [];
            
            % Create Initial Solution
            sol = empty_individual;
            sol.Position = randperm(obj.N_rob);
            sol.Cost = CostFunction(sol.Position);
            sol.Dock = obj.dock;
            
            % Initialize Best Solution Ever Found
            BestSol = sol;
            
            % Array to Hold Best Costs
            BestCost = zeros(MaxIt, 1);
            
            % Initialize Action Tabu Counters
            TC = zeros(nAction, 1);
            
%             opt_it = zeros(obj.CandLen, 1);  
            last_sol_i = 0;
            
            %% Tabu Search Main Loop
            
            for it = 1:MaxIt
                
%                 % Set maximum iteration number for the next soluion
%                 if opt_it(end)
%                     disp("Reach maximum length of candidate list.")
%                     disp("Found "+string(length(obj.solutions))...
%                             +" feasible solutions.");
% %                     disp(obj.solutions);
%                     disp("Exit searching ...")
%                     ExitCode = 1;
% %                     it = it - 1;
%                     break
%                 end
                if length(obj.N_it) > 1
%                     if opt_it(2) == 0   % length == 1
%                         prev_it = opt_it(opt_i-1);
%                     else                % length > 1
%                         prev_it = opt_it(opt_i-1) - opt_it(opt_i-2);
%                     end
%                     prev_it = opt_it(1);
                    mean_it = mean(obj.N_it);
                    if it - last_sol_i > mean_it * SearchMulti
%                         disp("Iteration end.");
                        disp("Total number of iterations: "+string(it));
                        disp("Found "+string(length(obj.solutions))...
                            +" feasible solutions.");
%                         disp(obj.solutions);
                        obj.ExitCode = 1;
%                         it = it - 1;
                        break
                    end
                end

                bestnewsol.Cost = inf;
                
                % Apply Actions
                for i = 1:nAction
                    if TC(i) == 0
                        [newsol.Position, newsol.Dock] = DoAction(sol.Position, sol.Dock, ActionList{i});
%                         if ActionList{i}(1) == 4
%                             % record rotation
%                             a = ActionList{i}(2:3);
%                             obj.rotation(a(1)) = rem(obj.rotation(a(1))+a(2),4);
%                         end
                        obj.dock = newsol.Dock;
                        newsol.Cost = CostFunction(newsol.Position);
%                         newsol.Rotation = obj.rotation;
                        newsol.ActionIndex = i;
            
                        if newsol.Cost <= bestnewsol.Cost
                            bestnewsol = newsol;
                        end
                    end
                end
                
                % Update Current Solution
                sol = bestnewsol;
                
                % Update Tabu List
                for i = 1:nAction
                    if i == bestnewsol.ActionIndex
                        TC(i) = TL;               % Add To Tabu List
                    else
                        TC(i) = max(TC(i)-1, 0);   % Reduce Tabu Counter
                    end
                end
                
                % Update Best Solution Ever Found
                if sol.Cost <= BestSol.Cost
                    BestSol = sol;
                end
                
                % Save Best Cost Ever Found
                BestCost(it) = BestSol.Cost;
                
                % Show Iteration Information
                if ~rem(it, 1000)
                    disp(['Iteration ' num2str(it) ': Best Cost = ' num2str(BestCost(it))]);
                end
                
            %     % Plot Best Solution
            %     figure(1);
            %     PlotSolution(BestSol.Position);
            %     pause(0.01);
                
                % If Global Minimum is Reached
                if BestCost(it) == 1 + obj.N_rob - obj.N_tar
                    add_newsol = true;
                    for i=1:length(obj.solutions)
                        temp_x = obj.solutions(i).Position;
                        if all(BestSol.Position == temp_x)
                            temp_dock = obj.solutions(i).Dock;
                            if all(all(BestSol.Dock == temp_dock))
                                add_newsol = false;
                            end
                        end
                    end
                    if add_newsol
                        obj.solutions = [obj.solutions BestSol];
                        if ~last_sol_i
                            obj.N_it = [obj.N_it, it];
                        else
                            obj.N_it = [obj.N_it, it - obj.N_it(end)];
                        end
                        last_sol_i = it;
%                         opt_it(opt_i) = it;
%                         opt_i = opt_i + 1;
                        disp("Iteration "+string(it)+" :");
                        disp("Found a feasible solution: ")
                        disp(BestSol);
                        disp("Dock: "); disp(BestSol.Dock);
                    end
                    obj.ExitCode = 2;
%                     break;
                end
                
            end
            
            if obj.ExitCode == 1
                it = it - 1;
            end

            BestCost = BestCost(1:it);
            cost = BestCost(end) - (obj.N_rob - obj.N_tar);
            
            %% Results
            
            % Print exit code
            obj.DispExitCode();

            % Find best solution for the candidates
            if obj.ExitCode
                obj.FindBestSolution();
                obj.DispBestSolCode();
    
                disp(' ');
                disp('Best Solution:');
                disp("(Robot No., Target No.)");
                x = obj.opt_sol.Position;
                y = 1:obj.N_rob;
                for j = 1:obj.N_rob
    %                 disp(['Queen #' num2str(j) ' at (' num2str(x(j)) ', ' num2str(y(j)) ')']);
                    disp(['(' num2str(x(j)) ', ' num2str(y(j)) ')']);
                end
                sol = obj.opt_sol.Position;
                dock = obj.opt_sol.Dock;
                disp("Dock: "); disp(dock);
            else   % No feasible solution
                obj.opt_sol = BestSol;
                sol = []; dock = [];
            end
        end

        function FindBestSolution(obj)
            temp_sol = obj.solutions;
            if length(temp_sol) == 1
                obj.opt_sol = temp_sol;
                obj.BestSolCode = 0;
                return
            end
            % Step 1: select shortest solutions
            temp_sol = obj.FindShortestSolution(temp_sol);
            if length(temp_sol) == 1
                obj.opt_sol = temp_sol;
                obj.BestSolCode = 1;
                return
            elseif length(temp_sol) > 1
                % Step 2: find min extension steps from the shortest dist
                % solutions
                temp_sol = obj.FindMostEfficientSolution(temp_sol);
                if length(temp_sol) == 1
                    obj.opt_sol = temp_sol;
                    obj.BestSolCode = 2;
                    return
                elseif length(temp_sol) > 1
                    % Step 3: find total number of connected dock joints
                    % and select solution(s) with maximal connected number.
                    temp_sol = obj.FindMaxConnectedSolution(temp_sol);
                    if length(temp_sol) == 1
                        obj.opt_sol = temp_sol;
                        obj.BestSolCode = 3;
                        return
                    elseif length(temp_sol) > 1
                        % Step 4: randomly select a solution from the
                        % temp_sol.
                        obj.opt_sol = obj.RandomlySelectASolution(temp_sol);
                        obj.BestSolCode = 4;
                        return
                    else
                        error("Error occurs when finding the max connected solution.");
                    end
                else
                    error("Error occurs when finding the most efficient solution.");
                end
            else
                error("Error occurs when finding the shortest solution.");
            end
%             obj.rotation = obj.solutions.Rotation;
        end
        
        function selected_sol = FindShortestSolution(obj, temp_sol)
            dist_sum = inf; best_i = [];
            for i=1:length(temp_sol)
                x = temp_sol(i).Position;
                temp_dist = 0;
                for j=1:obj.N_tar
                    t = obj.point(j,:);
                    r = obj.rob_loc(x(j),:);
                    d = obj.getManhattanDist(t, r);
                    temp_dist = temp_dist + d;
                end
                if temp_dist < dist_sum
                    dist_sum = temp_dist;
                    best_i = i;
                elseif temp_dist == dist_sum
                    best_i = [best_i i];
                end
            end
            selected_sol = temp_sol(best_i);
            disp("--------------- Step 1 ---------------");
            disp("Select "+string(length(selected_sol))+" solutions "+...
                "with shortest sum of distance from "+...
                string(length(temp_sol))+" feasible solutions.");
            disp("Shortest total distance: "+string(dist_sum));
        end
        
        function selected_sol = FindMostEfficientSolution(obj, temp_sol)
            % For every solution in temp_sol, initial an extension object
            % and build the corresponding assembly tree, select the
            % solution(s) with minimal assembly tree depth.
            N_sol = length(temp_sol);
            tars = obj.setTargets();
            d_min = Inf;  best_i = [];
            for i=1:N_sol
                assignment = temp_sol(i).Position;
                temp_dock = temp_sol(i).Dock;
                tars.setDisplayIDandDock(assignment, temp_dock);
                d = obj.getExtensionDepth(tars);
                if d < d_min
                    d_min = d;
                    best_i = i;
                elseif d == d_min
                    best_i = [best_i i];
                end
            end
            selected_sol = temp_sol(best_i);
            disp("--------------- Step 2 ---------------");
            disp("Select "+string(length(selected_sol))+" solutions "+...
                "with minimal extension depth from "+...
                string(N_sol)+" feasible solutions with shortest total distance.");
            disp("Minimal extension depth: "+string(d_min));
        end
        
        function selected_sol = FindMaxConnectedSolution(obj, temp_sol)
            N_sol = length(temp_sol);
            N_max_connect = 0;  best_i = [];
            for i=1:N_sol
                temp_dock = obj.rearrangeDock(temp_sol(i));
                graph = CreateAdjacentMatrix(obj.point, temp_dock, ...
                    obj.isDockIden);
                [edge, ~] = find(graph);
                N_connect = length(edge)/2;
                if N_connect > N_max_connect
                    N_max_connect = N_connect;
                    best_i = i;
                elseif N_connect == N_max_connect
                    best_i = [best_i i];
                end
            end
            selected_sol = temp_sol(best_i);
            disp("--------------- Step 3 ---------------");
            disp("Select "+string(length(selected_sol))+" solutions "+...
                "with maximal connection number from "+string(N_sol)+...
                " solutions with shortest total distance and minimal extension steps.");
            disp("Maximal connection number: "+string(N_max_connect));
        end
        
        function selected_sol = RandomlySelectASolution(~, temp_sol)
            if length(temp_sol) <= 1
                error("Error occurs when randomly select a solution.");
            end
            N_sol = length(temp_sol);
            idx = randi(N_sol);
            selected_sol = temp_sol(idx);
            disp("--------------- Step 4 ---------------");
            disp("Randomly select the "+string(idx)+"th solution as "+...
                "the best solution from "+string(N_sol)+" solutions.");
        end
        
        function d = getManhattanDist(~, p1, p2)
            d = sum(abs(p1 - p2));
        end
        
        function depth = getExtensionDepth(~, tar, ext_c, ext_l)
%             fintar = obj.setTargets();
            if nargin == 2
                ext_c = tar.TargetList(1).Location;
                ext_l = 3;
            elseif nargin == 3
                ext_l = 3;
            end
            ext = Extension(tar);
            t = ext.TargetToTree(ext_c, ext_l);
            depth = t.depth;
        end
        
        function fintar = setTargets(obj)
            locs = obj.point;
            [l, ~] = size(locs);
            Tars = [];
            for i = 1:l
                Tars =[Tars; targetPoint(i, locs(i, :))];
            end
            fintar = targetGroup(Tars);
        end
        
        function new_dock = rearrangeDock(~, sol)
            l = length(sol.Position);
            new_dock = zeros(l, 4);
            for i=1:l
                new_dock(i,:) = sol.Dock(sol.Position(i),:);
            end
        end

        function z = MyCost(obj, x)
            % x determine the dock sequence
            dock_swap = zeros(obj.N_rob, 4);
            for i=1:obj.N_rob
                dock_swap(i,:) = obj.dock(x(i),:);
            end
%             dock_swap = [obj.dock(x(1),:);obj.dock(x(2),:);...
%                         obj.dock(x(3),:);obj.dock(x(4),:)];
        
            z = ConnectionCheck(obj.point, dock_swap, obj.isDockIden);
        end

        function DispExitCode(obj)
            exitcode = obj.ExitCode;
            disp("Exit code: "+string(exitcode));

            switch exitcode
                
                case 0
                    % reach maximum iteration times and have not found
                    % feasible solution
                    disp("Maximum number of iterations has reached,"+...
                        " but faied to find any feasible solution subject"+...
                        " to the user input.");

%                 case 1
%                     % find sufficient candidates
%                     disp("Candidate list has been filled up. The program"+...
%                         " considers the number of candidates is sufficient.");
%                     disp("Adjust the CandLen parameter if more/less"+...
%                         " candidates are desired.");

                case 1
                    % hard to find next candidate
                    disp("Maximum number of iteration for searching the"+...
                        " next candidate has reached.");
                    disp("The CandLen paremeter could be too large "+...
                        "subject to the user input.");

                case 2
                    % reach maximum iteration times and found some feasible
                    % solutions
                    disp("Maximum number of iterations has reached.");
                    disp("Found "+string(length(obj.solutions))...
                            +" feasible solutions.");
                    disp("Consider to tune the parameters CandLen and"+...
                        " SearchMulti to improve the program efficiency.");

            end
        end
        
        function DispBestSolCode(obj)
            disp("Best sol code: "+string(obj.BestSolCode));
            switch obj.BestSolCode
                case -1
                    disp("No feasible solution.");
                case 0
                    disp("Only 1 feasible solution, which by default is "+...
                        "the optimal solution that has been found.");
                case 1
                    disp("Best solution selected by shortest distance criterion.");
                case 2
                    disp("Best solution selected by minimal extension steps"+...
                        " among the solutions with shortest total distance.");
                case 3
                    disp("Best solution selected by maximal connection number"+...
                        " among the solutions with minimal extension steps "+...
                        "and shortest total distance.");
                case 4
                    disp("Best solution randomly selected from the solutions"+...
                        " with maximal connection number, minimal extension steps"+...
                        ", and shortest total distance.");
            end
        end
    end
end

