classdef PIDs_controller < handle
    properties
        x_pid
        y_pid
        z_pid
        yaw_pid
        
    end
    
    methods
        function self = PIDs_controller()
           self.x_pid = x_PID();
           self.y_pid = y_PID();
           self.z_pid = z_PID();
           self.yaw_pid = yaw_PID();
      
        end
        
        function self = compute(self, yaw,yaw_ref,yaw_rate)
            % manage state variables into the right places
            % call the controllers
            % return the tau
      
        end

        function self = reset(self)
           % reset every controller
        end

      
    end
end
