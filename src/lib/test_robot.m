% Robot planning test
clear

%%% Generating a MAP
%1 represent an object that the path cannot penetrate, zero is a free path
MAP=false(128,140);
MAP(1:64,1)=1;
MAP(120,3:100)=1;
MAP(125:128,40:60)=1;
MAP(120:128,100:120)=1;
MAP(126,100:118)=0;
MAP(120:126,118)=0;
MAP(100:120,100)=1;
MAP(114:124,112:118)=0;
MAP(1,1:128)=1;
MAP(128,1:128)=1;
MAP(100,1:130)=1;
MAP(50,28:128)=1;
MAP(20:30,50)=1;
MAP(1:128,1)=1;
MAP(1:65,128)=1;
MAP(1,1:128)=1;
MAP(128,1:128)=1;
MAP(10,1:50)=1;
MAP(25,1:50)=1;
MAP(40,40:50)=1;
MAP(40,40:45)=1;
MAP(80,20:40)=1;
MAP(80:100,40)=1;
MAP(80:100,120)=1;
MAP(120:122,120:122)=1;
MAP(120:122,20:25)=1;
MAP(120:122,10:11)=1;
MAP(125:128,10:11)=1;
MAP(100:110,30:40)=1;
MAP(1:20,100:128)=1;
MAP(10:20,80:128)=1;
MAP(20:40,80:90)=1;
MAP(1:40,90:90)=1;
MAP(100:105,70:80)=1;
% imagesc(MAP);
% colormap(flipud(gray));
%%
MAP = MAP.';

% Initial g map
[Nx, Ny] = size(MAP);
gmap = map(Nx, Ny);
gmap.obstacleMap = MAP;
gmap.setDist("robot", 3, "module", 3);

%Start Positions
robots(1) = robot(1, [117, 124, 0], gmap);
% robots(1) = robot(1, [125, 65, 0]);
robots(1).CognMap = MAP;
robots(1).Goal = [116, 123, 0];
% robots(1).Goal = [80, 110, 0];
p = robots(1).Astar();

% imagesc(flip(MAP))
imagesc(MAP.');
colormap(flipud(gray));
hold on;
plot(p(:, 1), p(:, 2));

% while robots(1).move()
%     disp(robots(1).Location);
% end
