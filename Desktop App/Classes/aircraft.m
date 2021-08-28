classdef aircraft < handle
    %UNTITLED4 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        RawPatchData
        ScaledPatchData
        AircraftLength
        Factor
        Color
        Position
        Attitude
        OutPatchData
    end
    
    methods
        function obj = aircraft
            %UNTITLED4 Construct an instance of this class
            %   Detailed explanation goes here
            obj.RawPatchData = stlread('Extra 300.stl');
            obj.AircraftLength = 10;
            obj.Factor = (max(obj.RawPatchData.vertices(:,1))+abs(min(obj.RawPatchData.vertices(:,1))))/obj.AircraftLength;
            obj.ScaledPatchData = obj.RawPatchData;
            obj.ScaledPatchData.vertices = obj.RawPatchData.vertices/obj.Factor;
            obj.OutPatchData = obj.ScaledPatchData;
            obj.Position = [0,0,0];
        end
    end
    
    methods
        function moveAircraft(obj,position)%añadir attitude
            obj.Position = position;
            %obj.Attitude = attitude;
            obj.OutPatchData.vertices(:,1) = obj.ScaledPatchData.vertices(:,1) + position(1);
            obj.OutPatchData.vertices(:,2) = obj.ScaledPatchData.vertices(:,2) + position(2);
            obj.OutPatchData.vertices(:,3) = obj.ScaledPatchData.vertices(:,3) + position(3);
        end
    end
end

