%% File: supplyment_code.m
% Read the initial dock joint positions from the data.mat files and add the
% information to logging files for experiments.
% The script was written since the original dock information had not been
% recorded in the log.

% addpath('lib/log/hardware_exp');

shape_list = ["S", "T", "L", "1", "q"];
p = 'lib/log/hardware_exp/';
for i=1:length(shape_list)
    for j = 1:3
        folder = p + shape_list(i) + "-4t-4r-" + string(j) + "/";
        for k = 1:4
            file = folder + string(k) + ".txt";
            file_ = folder + string(k) + "_.txt";
            data = folder + "data.mat";
            % load variable
            load(data, 'Robot_Dock0');
            % todo rewrite files
            fid = fopen(file, 'a+');
            fid_ = fopen(file_, 'a+');
            fprintf(fid, "\n");
            fwrite(fid, "dock_origin: [");
            fwrite(fid, strjoin(string(Robot_Dock0(k, :)), ", "));
            fprintf(fid, "]\n");
            
            fprintf(fid_, "\n");
            fwrite(fid_, "dock_origin: [");
            fwrite(fid_, strjoin(string(Robot_Dock0(k, :)), ", "));
            fprintf(fid_, "]\n");
            fclose(fid); fclose(fid_);
        end
    end
end
