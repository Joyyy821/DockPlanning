% ÓÃ×÷»­Í¼£¨assembly tree£©

N = 6; a = 3; b = 2;
Tars = [];

for i=1:N
    m = ceil(i/a);
    n = i - (m-1) * a;
    Tars = [Tars; targetPoint(i, [m+2,n+4])];
end

gmap = map(10, 10);
t = Extension(targetGroup(Tars), gmap);
t.TargetToTree([3, 6], 3);
t.showTree();
t.showExtension();
