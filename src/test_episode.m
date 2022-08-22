%% test episode
clc; clear variables; close all;

ep = cell(1,3);
for i=1:3
    ep{i} = Episode("e", i);
    ep{i}.runExp();
end
