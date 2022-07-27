function dock = DoRotation(d, i1, i2)
%DOROTATION Rotation the element at i1 index with i2*45 degrees
%   Input: dock matrix (p), rotation index (i1), rotation command (i2)
%   Output: new dock matrix (q)
    dock_i1 = d(i1, :);
    dock = d; 
    switch i2
        case 1
            dock(i1, :) = dock_i1([3 4 2 1]);
        case 2
            dock(i1, :) = dock_i1([2 1 4 3]);
        case 3
            dock(i1, :) = dock_i1([4 3 1 2]);
    end
end