classdef module < handle
    %MODULE Represent the modules that robots will used for construction.
    %   The modules:
    %   (1) are identical and unacuated;
    %   (2) have fixed locations if attached to robot/bank/growing
    %   structure; 
    %   (3) have uncertain locations if not attached (because of the drift
    %   of water).
    
    properties (Access = public)
        % Can be directly modified by the user.
        ID            (1, 1) int32 {mustBeNonnegative} % ID of the module
        Status        (1, 1) logical % Attached (1) / Unattached (0)
        Loc           (1, 3) double  % Current module location [x, y, z]
    end
    
    methods
        function obj = module(id, loc)
            %MODULE Constructor
            %   1. Initialize module No.1 at location [1, 2, 0]:
            %   m = module(1, [1, 2, 0]);
            %   2. Initialize module No.2 at default location [0, 0, 0]:
            %   m = module(2);
            
            obj.Status = 0;  % Module should be floating at initial.
            if nargin == 0
                obj.ID = 0;
                obj.Loc = [0, 0, 0];
            end
            if nargin >= 1
                obj.ID = id;
                obj.Loc = [0, 0, 0];
            end
            if nargin >= 2
                obj.Loc = loc;
            end
        end
        
%         function outputArg = method1(obj,inputArg)
%             %METHOD1 此处显示有关此方法的摘要
%             %   此处显示详细说明
%             outputArg = obj.Property1 + inputArg;
%         end
    end
end

