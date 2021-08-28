classdef flightData < handle
    %FLIGHTDATA stores raw flight data coming from the device
    %   Detailed explanation goes here
    
    properties (Access = public)
        Pilot
        FlightTrack
        AircraftModel
    end
    
    properties (GetAccess = public, SetAccess = private)
        SensorData    
        Date
        StartTime
        Duration
    end
    
    methods
        function obj = flightData
            if nargin > 0 % in order to allow object matrix of this class
            end
        end
        
        function set.SensorData(obj,sensorData)
            % add functions to load sensor data, pilot name, model...
            obj.Date = extractDate(obj);
            obj.StartTime = extractStartTime(obj);
            obj.Duration = calcDuration(obj);
        end
        
        function date = extractDate(obj)
        end
        
        function startTime = extractStartTime(obj)
        end
        
        function duration = calcDuration(obj)
        end
    end
end

