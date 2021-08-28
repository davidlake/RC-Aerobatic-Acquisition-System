classdef flight < flightData
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Aircraft
        GraphData
    end
    
    methods
        function obj = flight(~)
            obj = obj@flightData;
            obj.GraphData = graphData;
        end
    end
    
    methods
        function proccesRawSensorData(obj)
        end
        function parseFlightFileData(obj)
        end
        function changeDeltaT(obj)
        end
        function generateGraphData(obj)
        end
    end
end

