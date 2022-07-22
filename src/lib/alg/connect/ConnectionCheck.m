function con_num = ConnectionCheck(point, dock)
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
        if dock(i,1) == 1 % up
            dock_state(1,2,i) = 1; 
        end
        if dock(i,2) == 1 % down
            dock_state(3,2,i) = 1; 
        end
        if dock(i,3) == 1 % left
            dock_state(2,1,i) = 1; 
        end
        if dock(i,4) == 1 % right
            dock_state(2,3,i) = 1; 
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
    matrix_adjacent = zeros(dock_len, dock_len);
    for j = 1:point_len
        row = point_position(j,1);
        col = point_position(j,2);
        index = target_map(row, col);
        %
        if row+3 < 3*map_size(1) % up
            if target_map(row+1, col) == 1 && target_map(row+2, col) == 1
                matrix_adjacent(j,target_map(row+3, col)) = 1;
            end
        end
        if row-3 > 0 % down
            if target_map(row-1, col) == 1 && target_map(row-2, col) == 1
                matrix_adjacent(j,target_map(row-3, col)) = 1;
            end
        end
        if col-3 > 0 % left
            if target_map(row, col-1) == 1 && target_map(row, col-2) == 1
                matrix_adjacent(j,target_map(row, col-3)) = 1;
            end
        end
        if col+3 < 3*map_size(2) % right
            if target_map(row, col+1) == 1 && target_map(row, col+2) == 1
                matrix_adjacent(j,target_map(row, col+3)) = 1;
            end
        end
    end
    
    %
    CG=CellG(matrix_adjacent,1);
    con_num = length(CG);
end