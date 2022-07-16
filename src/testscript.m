% ÓÃ×÷»­Í¼£¨assembly tree£©

N = 9; a = 3; b = 3;
% N = 8; a = 2; b = 4;
Tars = [];

for i=1:N
    m = ceil(i/a);
    n = i - (m-1) * a;
    Tars = [Tars; targetPoint(i, [m+1,n+4])];
end
all_tar = targetGroup(Tars);

gmap = map([20, 20]);
obstacle_locs = [1, 5; 1, 6; 1, 7];
obstacles = obstacle(obstacle_locs, gmap, true);

% % Modules
% module_locs = [2, 10; 5, 9; 6, 7; 8, 10; 10, 7; 8, 8; 8, 2; 9, 4; 10, 1];
% for i = 1:N
%     modules(i) = module(i, [module_locs(i, :), 0], gmap);
% end
% 
% % Robots
% robot_locs = [1, 1; 2, 1; 3, 1];
% % robot_locs = [1, 14; 4, 14; 9, 14; 12, 14];
% for i = 1:3
%     robots(i) = robot(i, [robot_locs(i, :), 0], gmap);
% end

% Current target
c_tar = targetGroup(copy(Tars));
% c_tar.TargetList(4).Location = [6, 2];
% c_tar.TargetList(5).Location = [6, 6];
% c_tar.TargetList(6).Location = [6, 7]; %[6, 10];
% % c_tar.TargetList(7).Location = [10, 2];
% c_tar.TargetList(7).Location = [7, 2];
% c_tar.TargetList(8).Location = [7, 6]; % [10, 6];
% c_tar.TargetList(9).Location = [7, 7]; % [10, 10];

display = display2D(gmap.mapSize, "FinalTarget", all_tar, ...
                                "CurrentTarget", c_tar, ...
                                "Obstacle", obstacles.Locations);
% display = display2D(gmap.mapSize, "FinalTarget", all_tar, ...
%                                 "Robot", robots, ...
%                                 "Module", modules, ...
%                                 "Obstacle", obstacles.Locations);
t = Extension(all_tar, gmap);
t.TargetToTree([2, 6], 3);
% t.showTree();
% pause(6);
t.showExtension(display, 1);
% [o, sr] = t.genConstructOrder(3);
