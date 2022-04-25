% ÓÃ×÷»­Í¼£¨assembly tree£©

N = 9; a = 3; b = 3;
Tars = [];

for i=1:N
    m = ceil(i/a);
    n = i - (m-1) * a;
    Tars = [Tars; targetPoint(i, [m+1,n+4])];
end
all_tar = targetGroup(Tars);

gmap = map(10, 10);
obstacle_locs = [1, 5; 1, 6; 1, 7];
obstacles = obstacle(obstacle_locs, gmap, true);
display = display2D(gmap.mapSize, "FinalTarget", all_tar, ...
                                "Obstacle", obstacles.Locations);
t = Extension(all_tar, gmap);
t.TargetToTree([2, 6], 3);
t.showTree();
t.showExtension(display);
[o, sr] = t.genConstructOrder(3);
