classdef (ConstructOnLoad) NewOdomEventData < event.EventData
   properties
      agent_number
      position
      orientation
   end
   
   methods
      function data = NewOdomEventData(agent_number, position, orientation)
         data.agent_number = agent_number;
         data.position = position;
         data.orientation = orientation;
      end
   end
end

