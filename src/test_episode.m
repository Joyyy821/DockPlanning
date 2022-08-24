%% test episode
clc; clear variables; close all;
addpath('lib');
addpath('lib/display'); addpath('lib/log');
addpath('lib/alg'); addpath('lib/alg/connect');

ep = cell(1,3);
for i=2:3
    ep{i} = Episode("e", i);
    ep{i}.runExp();
end
