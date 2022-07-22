classdef TabuSearch < handle 
    %TABUSEARCH Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        point    % target positions
        dock     % dock status
        N_tar
        N_rob
    end
    
    methods
        function obj = TabuSearch(init_point,init_dock)
            %TABUSEARCH Construct an instance of this class
            %   Detailed explanation goes here
            if nargin == 0
                obj.point = [4,4;5,4;5,5;6,5]; % x, y
                obj.dock = [0,0,0,1;...
                            0,1,0,1;...
                            0,0,1,0;...
                            1,0,1,0]; % up, down, left, right
            elseif nargin == 2
                obj.point = init_point;
                obj.dock = init_dock;
            end
            [obj.N_tar, ~] = size(obj.point);
            [obj.N_rob, ~] = size(obj.dock);
        end

        function setTS(obj, pt, d)
            obj.point = pt;
            obj.dock = d;
            [obj.N_tar, ~] = size(obj.point);
            [obj.N_rob, ~] = size(obj.dock);
        end
        
        function [sol, cost] = search(obj)
            %% Problem Definition
            
            CostFunction = @(p) obj.MyCost(p);    % Cost Function
            
            [nQueen, ~] = size(obj.point);   % Number of Queens
            
            ActionList = CreatePermActionList(nQueen);    % Action List
            
            nAction = numel(ActionList);              % Number of Actions
            
            
            %% Tabu Search Parameters
            
            MaxIt = 50;                      % Maximum Number of Iterations
            
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
                        newsol.Position = DoAction(sol.Position, ActionList{i});
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
                disp(['Iteration ' num2str(it) ': Best Cost = ' num2str(BestCost(it))]);
                
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

