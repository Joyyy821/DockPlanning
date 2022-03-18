classdef AssembleGroup < handle
    %ASSEMBLEGROUP 此处显示有关此类的摘要
    %   此处显示详细说明
    
    properties
        LeadRobot          robot
        modules            moduleGroup
        attachModuleID     int32
        GlobalMap          map       % "pointer" to a global map object
        dockFlag           logical
    end
    
    methods
        function obj = AssembleGroup(rob, gmap)
            %ASSEMBLEGROUP 构造此类的实例
            %   此处显示详细说明
            if nargin == 1
                obj.LeadRobot = rob;
            end
            if nargin == 2
                obj.GlobalMap = gmap;
            end
            
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 此处显示有关此方法的摘要
            %   此处显示详细说明
            outputArg = obj.Property1 + inputArg;
        end
    end
end

