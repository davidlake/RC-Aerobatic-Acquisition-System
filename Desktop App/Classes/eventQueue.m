classdef eventQueue < handle
    
    properties (Access = private)
        Queue
        Idx
        Size
    end
    
    methods
        
        function obj = eventQueue(size)
            obj.Size = size;
            obj.Queue = strings(1,size); %queue initialization and allocation
            obj.Idx = 0;
        end

        function addEvent(obj, eventTag)
            if obj.Idx < obj.Size
                obj.Idx = obj.Idx + 1;
                obj.Queue(obj.Idx) = eventTag;
            end
        end
        
        function eventTag = processEvent(obj)
            switch obj.Idx
                case 0
                    eventTag = "";
                case 1
                    eventTag = obj.Queue(1);
                    obj.Queue(1) = "";
                    obj.Idx = obj.Idx - 1;
                otherwise
                    eventTag = obj.Queue(1);
                    obj.Queue = [obj.Queue(2:obj.Idx) strings(1,obj.Size-obj.Idx-1)];
                    obj.Idx = obj.Idx - 1;                    
            end
        end
        
    end
end

