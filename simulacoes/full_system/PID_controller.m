classdef PID_controller < handle
    properties
        %parameters:
        K_p
        K_i
        K_d
        K_a
        K_e
        alpha
        home_D
        R 
        Dt

        %variables:
        state
        yaw_corr
        output
        
    end
    
    methods
        function self = PID_controller(Dt)
            self.K_delta = 10;
            self.K_phi = 1;
            self.K_e=-1;
            self.alpha = 10;
            self.home_D = 20;
            self.R = 20;
            self.Dt = Dt;

            self.state=1;
            self.yaw_corr = 0;
            self.output =[0;0];
        end
        
        function self = compute(self, x, y)
            
           
        end

      
    end
end
