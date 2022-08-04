function con_num = ConnectionCheck(point, dock, is_dock_identical)
    if nargin == 2
        is_dock_identical = isempty(find(dock==2,1));
    end
    if is_dock_identical && ~isempty(find(dock==2,1))
        warning("The input dock joints are not identical but indicated"+...
            " as identical. The dock joints will be converted to uniform version.");
        dock = dock | zeros(size(dock));
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
    if is_dock_identical
        connect_val = 2;
    else
        connect_val = 3;
    end
    [dock_len, ~] = size(dock);
    matrix_adjacent = zeros(dock_len, dock_len);
    for j = 1:point_len
        row = point_position(j,1);
        col = point_position(j,2);
        index = target_map(row, col);
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
    
    %
    CG=CellG(matrix_adjacent,1);
    con_num = length(CG);
end