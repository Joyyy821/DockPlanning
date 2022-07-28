classdef TabuSearch < handle 
    %TABUSEARCH Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        point    % target positions
        dock     % dock status
        rob_loc  % robot initial locations
        rotation % rotation command
        N_tar
        N_rob
        solutions  % Feasible solutions
    end
    
    methods
        function obj = TabuSearch(init_point,init_dock, init_loc)
            %TABUSEARCH Construct an instance of this class
            %   Detailed explanation goes here
            if nargin == 0
                obj.point = [4,4;5,4;5,5;6,5]; % x, y
                obj.dock = [1,0,0,0;...
                            1,0,1,0;...
                            0,1,0,0;...
                            1,0,1,0;...
                            1,1,0,0
                            ]; % up, down, left, right
                obj.rob_loc = [1, 1; 4, 1; 7, 1; 10, 1; 13, 1];
%                 obj.dock = [0,0,0,1;...
%                             0,1,0,1;...
%                             0,0,1,0;...
%                             1,0,1,0]; % up, down, left, right
            elseif nargin == 3
                obj.point = init_point;
                obj.dock = init_dock;
                obj.rob_loc = init_loc;
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
        
        function [sol, rotation, cost] = search(obj)
            obj.rotation = zeros(obj.N_rob, 1);
            obj.solutions = [];
            %% All-zero Dock Input Checking
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
            %% Problem Definition
            
            CostFunction = @(p) obj.MyCost(p);    % Cost Function
            
            [nQueen, ~] = size(obj.dock);   % Number of Queens
            
            ActionList = CreatePermActionList(obj.dock);    % Action List
            
            nAction = numel(ActionList);              % Number of Actions
            
            
            %% Tabu Search Parameters
            
            MaxIt = 50000;                      % Maximum Number of Iterations
            
            TL = round(0.5*nAction);      % Tabu Length
            
            
            %% Initialization
            
            % Create Empty Individual Structure
            empty_individual.Position = [];
            empty_individual.Cost = [];
            empty_individual.Rotation = [];
            
            % Create Initial Solution
            sol = empty_individual;
            sol.Position = randperm(nQueen);
            sol.Cost = CostFunction(sol.Position);
            sol.Rotation = obj.rotation;
            
            % Initialize Best Solution Ever Found
            BestSol = sol;
            
            % Array to Hold Best Costs
            BestCost = zeros(MaxIt, 1);
            
            % Initialize Action Tabu Counters
            TC = zeros(nAction, 1);
            
            
            %% Tabu Search Main Loop

            opt_it = [];
            
            for it = 1:MaxIt
                
                % Set maximum iteration number for the next soluion
                if ~isempty(opt_it)
                    if length(opt_it) == 1
                        prev_it = opt_it(end);
                    elseif length(opt_it) > 1
                        prev_it = opt_it(end) - opt_it(end-1);
                    end
                    if it - opt_it(end) > prev_it * 2
                        disp("Iteration end. Found "+string(length(obj.solutions))...
                            +" feasible solutions.");
                        disp(obj.solutions);
                        break
                    end
                end

                bestnewsol.Cost = inf;
                
                % Apply Actions
                for i = 1:nAction
                    if TC(i) == 0
                        [newsol.Position, obj.dock] = DoAction(sol.Position, obj.dock, ActionList{i});
                        if ActionList{i}(1) == 4
                            % record rotation
                            a = ActionList{i}(2:3);
                            obj.rotation(a(1)) = rem(obj.rotation(a(1))+a(2),4);
                        end
                        newsol.Cost = CostFunction(newsol.Position);
                        newsol.Rotation = obj.rotation;
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
                if ~rem(it, 100)
                    disp(['Iteration ' num2str(it) ': Best Cost = ' num2str(BestCost(it))]);
                end
                
            %     % Plot Best Solution
            %     figure(1);
            %     PlotSolution(BestSol.Position);
            %     pause(0.01);
                
                % If Global Minimum is Reached
                if BestCost(it) == 1 + obj.N_rob - obj.N_tar
                    obj.solutions = [obj.solutions BestSol];
                    opt_it = [opt_it it];
                    disp("Found a feasible solution: ")
                    disp(BestSol);
%                     break;
                end
                
            end
            
            BestCost = BestCost(1:it);
            cost = BestCost(end) - (obj.N_rob - obj.N_tar);
            
            %% Results
            
            % Find best solution for the candidates
            obj.FindShortestSolution();

            disp(' ');
            disp('Best Solution:');
            x = obj.solutions.Position;
            y = 1:nQueen;
            for j = 1:nQueen
                disp(['Queen #' num2str(j) ' at (' num2str(x(j)) ', ' num2str(y(j)) ')']);
            end
            sol = x;
            rotation = obj.rotation;
        end

        function FindShortestSolution(obj)
            dist_sum = inf; best_i = 0;
            for i=1:length(obj.solutions)
                x = obj.solutions(i).Position;
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
                end
            end
            obj.solutions = obj.solutions(best_i);
            obj.rotation = obj.solutions.Rotation;
        end

        function d = getManhattanDist(~, p1, p2)
            d = sum(abs(p1 - p2));
        end

        function z = MyCost(obj, x)
            % x determine the dock sequence
            dock_swap = zeros(obj.N_rob, 4);
            for i=1:obj.N_rob
                dock_swap(i,:) = obj.dock(x(i),:);
            end
%             dock_swap = [obj.dock(x(1),:);obj.dock(x(2),:);...
%                         obj.dock(x(3),:);obj.dock(x(4),:)];
        
            z = ConnectionCheck(obj.point, dock_swap);
        end
    end
end

