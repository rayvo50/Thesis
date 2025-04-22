classdef docking_controller < handle
    % manages all the controllers for each stage of the docking manoeuvre

    properties
        phase;
        tau;
        approaching_controller;
        homing_controller;
        terminal_controller; 
        tau_max;
        tau_min;
        state;
        s;
        Phis;
        e;
        ref;
    end
    
    methods
        function self = docking_controller()
            self.phase = "homing";
            self.approaching_controller = nan;%approaching_controller();
            self.homing_controller = homing_controller();
            self.terminal_controller = nan;%terminal_controller();
            self.tau_max = [100;100;100;100];
            self.tau_min = [-100;-100;-100;-100];
            self.s = nan(4,1);
            self.e = nan(4,1);
            self.ref = nan(6,1);
        end
        
        function self = compute(self, x_hat, dvl, ahrs)
            self.tau = zeros(4,1);
            
            %TODO: implement state-machine switching with hysterisis
            
            if self.phase == "approaching"
                %TODO
                self.approaching_controller.compute(x_hat, dvl, ahrs);
                self.tau = self.approaching_controller.output;
                self.state = self.approaching_controller.state;
                
            elseif self.phase == "homing"
                self.homing_controller.compute(x_hat, dvl, ahrs);
                self.tau = self.homing_controller.output;
                self.state = self.homing_controller.state;
                self.s = self.homing_controller.s;
                self.Phis = self.homing_controller.Phis;
                self.e = self.homing_controller.e;
                self.ref = self.homing_controller.ref;

            elseif self.phase == "terminal"
                %TODO
                self.terminal_controller.compute(x_hat, dvl, ahrs);
                self.tau = self.terminal_controller.output;
                self.state = self.terminal_controller.state;
            end
            
            % Saturate ouput
            self.tau = min(max(self.tau,  self.tau_min), self.tau_max);

        end      
    end
end
