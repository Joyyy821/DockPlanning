function matrix_adjacent = CreateAdjacentMatrix(point,dock, is_dock_identical, matrix_type)
%CREATADJACENTMATRIX Adjacent matrix that shows the connectivity of targets
%   INPUTS
%   point: size (N_tar, 2), coordinates of all targets
%   dock: size (N_tar, 4), dock distributions
%   is_dock_identical: bool, whether universal docks or N/S docks
%   matrix_type: int, 1 - connection matrix, 2 - contact matrix, 3 - same
%   gender matrix for dock = 1, 4 - same gender matrix for dock = 2
%   (Note: matrix_type = 3 or 4 only applies if is_dock_identical == false)
%   matrix_type = 1 by default
%   OUTPUT
%   matrix_adjacent: size (N_tar, N_tar), adjacent matrix for connectivity
    
    if nargin == 3
        matrix_type = 1;  % Set default value
    elseif nargin == 4 && matrix_type >=3 && is_dock_identical
        error("No same gender mismatch for universal dock");
    end

    % find the max dimension and create the local map
    map_low_boundary = min(point);
    map_size = max(point)-map_low_boundary+1;
    map_size([1 2]) = map_size([2 1]);
    target_map = zeros(3.*map_size);
    
    % place the point
    [point_len, ~] = size(point); % num*dimension
    dock_state = zeros(3,3,point_len);
    point_position = [];
    for i = 1:point_len
        % make the dock matrix
        if dock(i,1) % up
            dock_state(1,2,i) = dock(i,1); 
        end
        if dock(i,2) % down
            dock_state(3,2,i) = dock(i,2); 
        end
        if dock(i,3) % left
            dock_state(2,1,i) = dock(i,3); 
        end
        if dock(i,4) % right
            dock_state(2,3,i) = dock(i,4); 
        end
        dock_state(2,2,i) = i; % place the point index
        
        % fill the map
        r = 3.*(map_size(1) - (point(i,2)-map_low_boundary(2))-1);
        c = 3.*(point(i,1)-map_low_boundary(1)); 
        target_map(r+1:r+3,c+1:c+3) = dock_state(:,:,i);
        point_position = [point_position; r+2,c+2];
    end
    
    % create the adjacent matrix
    [dock_len, ~] = size(dock);
    
    % Create adjacent matrix according to the input matrix_type
    switch matrix_type
        case 1
            % connection matrix
            if is_dock_identical
                connect_val = 2;
            else
                connect_val = 3;
            end
            matrix_adjacent = connectionMatrix(target_map, map_size, ...
                point_position, connect_val, dock_len, point_len);
        case 2
            % contact matrix
            matrix_adjacent = contactMatrix(target_map, map_size, ...
                point_position, dock_len, point_len);
        case 3
            % same gender (1) 
            gender = 1;
            matrix_adjacent = sameGenderMatrix(target_map, map_size, ...
                point_position, dock_len, point_len, gender);
        case 4
            % same gender (2)
            gender = 2;
            matrix_adjacent = sameGenderMatrix(target_map, map_size, ...
                point_position, dock_len, point_len, gender);
    end

end

function matrix_adjacent = connectionMatrix(target_map, map_size, ...
    point_position, connect_val, dock_len, point_len)
    matrix_adjacent = zeros(dock_len, dock_len);
    for j = 1:point_len
        row = point_position(j,1);
        col = point_position(j,2);
%         index = target_map(row, col);
        % 
        if row+3 < 3*map_size(1) % up
            if target_map(row+1, col) + target_map(row+2, col) == connect_val
                matrix_adjacent(j,target_map(row+3, col)) = 1;
            end
        end
        if row-3 > 0 % down
            if target_map(row-1, col) + target_map(row-2, col) == connect_val
                matrix_adjacent(j,target_map(row-3, col)) = 1;
            end
        end
        if col-3 > 0 % left
            if target_map(row, col-1) + target_map(row, col-2) == connect_val
                matrix_adjacent(j,target_map(row, col-3)) = 1;
            end
        end
        if col+3 < 3*map_size(2) % right
            if target_map(row, col+1) + target_map(row, col+2) == connect_val
                matrix_adjacent(j,target_map(row, col+3)) = 1;
            end
        end
    end
end

function matrix_adjacent = contactMatrix(target_map, map_size, ...
    point_position, dock_len, point_len)
    matrix_adjacent = zeros(dock_len, dock_len);
    for j = 1:point_len
        row = point_position(j,1);
        col = point_position(j,2);
%         index = target_map(row, col);
        % 
        if row+3 < 3*map_size(1) % up
            if target_map(row+3, col)
                matrix_adjacent(j,target_map(row+3, col)) = 1;
            end
        end
        if row-3 > 0 % down
            if target_map(row-3, col)
                matrix_adjacent(j,target_map(row-3, col)) = 1;
            end
        end
        if col-3 > 0 % left
            if target_map(row, col-3)
                matrix_adjacent(j,target_map(row, col-3)) = 1;
            end
        end
        if col+3 < 3*map_size(2) % right
            if target_map(row, col+3)
                matrix_adjacent(j,target_map(row, col+3)) = 1;
            end
        end
    end
end

function matrix_adjacent = sameGenderMatrix(target_map, map_size, ...
    point_position, dock_len, point_len, gender)
    matrix_adjacent = zeros(dock_len, dock_len);
    for j = 1:point_len
        row = point_position(j,1);
        col = point_position(j,2);
%         index = target_map(row, col);
        % 
        if row+3 < 3*map_size(1) % up
            if target_map(row+1, col) == gender && target_map(row+2, col) == gender
                matrix_adjacent(j,target_map(row+3, col)) = 1;
            end
        end
        if row-3 > 0 % down
            if target_map(row-1, col) == gender && target_map(row-2, col) == gender
                matrix_adjacent(j,target_map(row-3, col)) = 1;
            end
        end
        if col-3 > 0 % left
            if target_map(row, col-1) == gender && target_map(row, col-2) == gender
                matrix_adjacent(j,target_map(row, col-3)) = 1;
            end
        end
        if col+3 < 3*map_size(2) % right
            if target_map(row, col+1) == gender && target_map(row, col+2) == gender
                matrix_adjacent(j,target_map(row, col+3)) = 1;
            end
        end
    end
end

