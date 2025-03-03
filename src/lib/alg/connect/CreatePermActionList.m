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

function ActionList = CreatePermActionList(dock, isrotation)
    
    [n, ~] = size(dock);
    nSwap = n*(n-1)/2;
    nReversion = n*(n-1)/2;
    nInsertion = n^2;

    % add rotation
    nRotation = 3*n;
%     for i=1:n
%         if all(dock(i,:)==[1,1,0,0]) || all(dock(i,:)==[0,0,1,1])
%             nRotation = nRotation - 2;
%         elseif all(dock(i,:))
%             nRotation = nRotation - 3;
%         end
%     end

    nAction = nSwap+nReversion+nInsertion+nRotation;
    
    ActionList = cell(nAction, 1);
    
    c = 0;
    
    % Add SWAP
    for i = 1:n-1
        for j = i+1:n
            c = c+1;
            ActionList{c} = [1 i j];
        end
    end

    % Add REVERSION
    for i = 1:n-1
        for j = i+1:n
            if abs(i-j)>2
                c = c+1;
                ActionList{c} = [2 i j];
            end
        end
    end
    
    % Add Insertion
    for i = 1:n
        for j = 1:n
            if abs(i-j)>1
                c = c+1;
                ActionList{c} = [3 i j];
            end
        end
    end
    
    if isrotation
        % Add Rotation
        for i=1:n
            if all(dock(i,:)==[1,1,0,0]) || all(dock(i,:)==[0,0,1,1])
                c = c+1;
                ActionList{c} = [4 i 1];
            elseif all(dock(i,:))
                continue
            else
                for j=1:3
                    c = c+1;
                    ActionList{c} = [4 i j];
                end
            end
        end
    end
    
    ActionList = ActionList(1:c);

end