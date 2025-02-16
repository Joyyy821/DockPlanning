function matches = SolveLSAP(tar_locs, robot_locs)
%SOLVELSAP Assign robots to targets, minimize the sum of distances
%   INPUTS
%   tar_locs, robot_locs: size (N, 2)
%   OUTPUT
%   matches: size (N, 2), maps targets to robots
%   e.g., matches(1, :) = [3, 2] means that target No. 3 assigns to robot
%   No. 2
    [N_tar, ~] = size(tar_locs);
    [N_rob, ~] = size(robot_locs);
    matches = [];
    if N_tar ~= N_rob
        disp("WARN: Number of targets ("+string(N_tar)+") doesn't match with number of robots ("+str(N_rob)+").");
        return
    end

    % Construct the cost matrix
    Cost = zeros(N_tar, N_rob);
    for i=1:N_tar
        for j=1:N_rob
            Cost(i, j) = ManhattanDist(tar_locs(i, :), robot_locs(j, :));
        end
    end

    % Solve LSAP
    matches = matchpairs(Cost, 1e4);
end

function dist = ManhattanDist(tar_loc, robot_loc)
    dist = abs(robot_loc(1) - tar_loc(1)) + abs(robot_loc(2) - tar_loc(2));
end