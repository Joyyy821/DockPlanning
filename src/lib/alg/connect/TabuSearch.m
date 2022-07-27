classdef TabuSearch < handle 
    %TABUSEARCH Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        point    % target positions
        dock     % dock status
        rotation % rotation command
        N_tar
        N_rob
    end
    
    methods
        function obj = TabuSearch(init_point,init_dock)
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
%                 obj.dock = [0,0,0,1;...
%                             0,1,0,1;...
%                             0,0,1,0;...
%                             1,0,1,0]; % up, down, left, right
            elseif nargin == 2
                obj.point = init_point;
                obj.dock = init_dock;
            end
            [obj.N_tar, ~] = size(obj.point);
            [obj.N_rob, ~] = size(obj.dock);
            obj.rotation = zeros(obj.N_rob, 1);
        end

        function setTS(obj, pt, d)
            obj.point = pt;
            obj.dock = d;
            [obj.N_tar, ~] = size(obj.point);
            [obj.N_rob, ~] = size(obj.dock);
        end
        
        function [sol, rotation, cost] = search(obj)
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
            
            MaxIt = 5000;                      % Maximum Number of Iterations
            
            TL = round(0.5*nAction);      % Tabu Length
            
            
            %% Initialization
            
            % Create Empty Individual Structure
            empty_individual.Position = [];
            empty_individual.Cost = [];
            
            % Create Initial Solution
            sol = empty_individual;
            sol.Position = randperm(nQueen);
            sol.Cost = CostFunction(sol.Position);
            
            % Initialize Best Solution Ever Found
            BestSol = sol;
            
            % Array to Hold Best Costs
            BestCost = zeros(MaxIt, 1);
            
            % Initialize Action Tabu Counters
            TC = zeros(nAction, 1);
            
            
            %% Tabu Search Main Loop
            
            for it = 1:MaxIt
                
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
                if ~rem(it, 10)
                    disp(['Iteration ' num2str(it) ': Best Cost = ' num2str(BestCost(it))]);
                end
                
            %     % Plot Best Solution
            %     figure(1);
            %     PlotSolution(BestSol.Position);
            %     pause(0.01);
                
                % If Global Minimum is Reached
                if BestCost(it) == 1 + obj.N_rob - obj.N_tar
                    break;
                end
                
            end
            
            BestCost = BestCost(1:it);
            cost = BestCost(end) - (obj.N_rob - obj.N_tar);
            
            %% Results
            
            disp(' ');
            disp('Best Solution:');
            x = BestSol.Position;
            y = 1:nQueen;
            for j = 1:nQueen
                disp(['Queen #' num2str(j) ' at (' num2str(x(j)) ', ' num2str(y(j)) ')']);
            end
            sol = x;
            rotation = obj.rotation;
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

