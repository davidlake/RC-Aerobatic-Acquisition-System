classdef graphData < handle
    
    properties (Access = public)
        Time %n matrix
        Position %nx3 matrix
        Attitude %nx3 matrix
    end
    
    properties (GetAccess = public, SetAccess = private)
        DeltaT
        Duration % not exactly the same than flight object duration. This one is corrected to be a multiple of deltaT
        nElements
    end
    
    methods
        function obj = graphData
        end
    end
    
    methods
        function set.Time(obj,time)
            obj.Time = time;
            obj.DeltaT = time(2) - time(1);
            obj.nElements = numel(time);
            obj.Duration = time(end);
        end
    end
    
end

