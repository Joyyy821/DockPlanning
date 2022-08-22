function con_num = ConnectionCheck(point, dock, is_dock_identical)
    if nargin == 2
        is_dock_identical = isempty(find(dock==2,1));
    end
    if is_dock_identical && ~isempty(find(dock==2,1))
        warning("The input dock joints are not identical but indicated"+...
            " as identical. The dock joints will be converted to uniform version.");
        dock = dock | zeros(size(dock));
    end
    matrix_adjacent = CreateAdjacentMatrix(point, dock, is_dock_identical);
    %
    CG=CellG(matrix_adjacent,1);
    con_num = length(CG);
end