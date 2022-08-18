classdef TabuSearch < handle 
    %TABUSEARCH Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        point    % target positions
        dock     % dock status
        rob_loc  % robot initial locations
        isRotation % rotation command
        solutions  % Feasible solutions
        % parameters
        N_tar
        N_rob
        isDockIden
        CandLen = 1
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
            
            [nQueen, ~] = size(obj.dock);   % Number of Queens
            
            ActionList = CreatePermActionList(obj.dock, obj.isRotation);    % Action List
            
            nAction = numel(ActionList);              % Number of Actions
            
            
            %% Tabu Search Parameters
            
            MaxIt = 1e5;                      % Maximum Number of Iterations
            
            TL = round(0.5*nAction);      % Tabu Length
            
%             CandLen = 4;               % Maximum number of feasible solutions to be recorded

            SearchMulti = 500;           % Allowed multiple iteration times for next solution

            ExitCode = 0;             % Indicate why the program stops

            %% Initialization
            
            % Create Empty Individual Structure
            empty_individual.Position = [];
            empty_individual.Cost = [];
            empty_individual.Dock = [];
            
            % Create Initial Solution
            sol = empty_individual;
            sol.Position = randperm(nQueen);
            sol.Cost = CostFunction(sol.Position);
            sol.Dock = obj.dock;
            
            % Initialize Best Solution Ever Found
            BestSol = sol;
            
            % Array to Hold Best Costs
            BestCost = zeros(MaxIt, 1);
            
            % Initialize Action Tabu Counters
            TC = zeros(nAction, 1);
            
            opt_it = zeros(obj.CandLen, 1);  opt_i = 1;
            
            %% Tabu Search Main Loop
            
            for it = 1:MaxIt
                
                % Set maximum iteration number for the next soluion
                if opt_it(end)
                    disp("Reach maximum length of candidate list.")
                    disp("Found "+string(length(obj.solutions))...
                            +" feasible solutions.");
%                     disp(obj.solutions);
                    disp("Exit searching ...")
                    ExitCode = 1;
%                     it = it - 1;
                    break
                end
                if any(opt_it)
%                     if opt_it(2) == 0   % length == 1
%                         prev_it = opt_it(opt_i-1);
%                     else                % length > 1
%                         prev_it = opt_it(opt_i-1) - opt_it(opt_i-2);
%                     end
                    prev_it = opt_it(1);
                    if it - opt_it(opt_i-1) > prev_it * SearchMulti
%                         disp("Iteration end.");
                        disp("Total number of iterations: "+string(it));
                        disp("Found "+string(length(obj.solutions))...
                            +" feasible solutions.");
%                         disp(obj.solutions);
                        ExitCode = 2;
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
                            add_newsol = false;
                        end
                    end
                    if add_newsol
                        obj.solutions = [obj.solutions BestSol];
                        opt_it(opt_i) = it;
                        opt_i = opt_i + 1;
                        disp("Iteration "+string(it)+" :");
                        disp("Found a feasible solution: ")
                        disp(BestSol);
                        disp("Dock: "); disp(BestSol.Dock);
                    end
                    ExitCode = 3;
%                     break;
                end
                
            end
            
            if ExitCode == 1 || ExitCode == 2
                it = it - 1;
            end

            BestCost = BestCost(1:it);
            cost = BestCost(end) - (obj.N_rob - obj.N_tar);
            
            %% Results
            
            % Print exit code
            obj.DispExitCode(ExitCode);

            % Find best solution for the candidates
            if ExitCode
                obj.FindShortestSolution();
    
                disp(' ');
                disp('Best Solution:');
                disp("(Robot No., Target No.)");
                x = obj.solutions.Position;
                y = 1:nQueen;
                for j = 1:nQueen
    %                 disp(['Queen #' num2str(j) ' at (' num2str(x(j)) ', ' num2str(y(j)) ')']);
                    disp(['(' num2str(x(j)) ', ' num2str(y(j)) ')']);
                end
                sol = obj.solutions.Position;
                dock = obj.solutions.Dock;
                disp("Dock: "); disp(dock);
            else
                sol = []; dock = [];
            end
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
%             obj.rotation = obj.solutions.Rotation;
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
        
            z = ConnectionCheck(obj.point, dock_swap, obj.isDockIden);
        end

        function DispExitCode(obj, exitcode)
            disp("Exit code: "+string(exitcode));

            switch exitcode
                
                case 0
                    % reach maximum iteration times and have not found
                    % feasible solution
                    disp("Maximum number of iterations has reached,"+...
                        " but faied to find any feasible solution subject"+...
                        " to the user input.");

                case 1
                    % find sufficient candidates
                    disp("Candidate list has been filled up. The program"+...
                        " considers the number of candidates is sufficient.");
                    disp("Adjust the CandLen parameter if more/less"+...
                        " candidates are desired.");

                case 2
                    % hard to find next candidate
                    disp("Maximum number of iteration for searching the"+...
                        " next candidate has reached.");
                    disp("The CandLen paremeter could be too large "+...
                        "subject to the user input.");

                case 3
                    % reach maximum iteration times and found some feasible
                    % solutions
                    disp("Maximum number of iterations has reached.");
                    disp("Found "+string(length(obj.solutions))...
                            +" feasible solutions.");
                    disp("Consider to tune the parameters CandLen and"+...
                        " SearchMulti to improve the program efficiency.");

            end
        end
    end
end

