%
% Copyright (c) 2015, Mostapha Kalami Heris & Yarpiz (www.yarpiz.com)
% All rights reserved. Please read the "LICENSE" file for license terms.
%
% Project Code: YPEA116
% Project Title: Implementation of Tabu Search for n-Queens Problem
% Publisher: Yarpiz (www.yarpiz.com)
% 
% Developer: Mostapha Kalami Heris (Member of Yarpiz Team)
% 
% Cite as:
% Mostapha Kalami Heris, Tabu Search (TS) in MATLAB (URL: https://yarpiz.com/243/ypea116-tabu-search), Yarpiz, 2015.
% 
% Contact Info: sm.kalami@gmail.com, info@yarpiz.com
%

% ts;

% A test script

% Initialization with default target and robot settings.
point = [4,4;5,4;5,5;6,5]; % x, y
dock = [1,0,0,0;...
        1,0,1,0;...
        0,1,0,0;...
        1,0,1,0
        ]; % up, down, left, right
rob_loc = [1, 1; 4, 1; 7, 1; 10, 1];
myts = TabuSearch(point, dock, rob_loc, true);
% Searching
[sol, dock, cost] = myts.search();
disp("Assignment: "); disp(sol);
disp("Dock: "); disp(dock);
disp("Cost: "); disp(cost);
