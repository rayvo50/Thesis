classdef ua_controller < handle
    properties
        %parameters:
        K_delta
        K_phi
        K_e
        alpha
        home_D
        R 
        Dt

        %variables:
        state
        yaw_corr
        output

        debug
        
    end
    
    methods
        function self = ua_controller(Dt)
            self.K_delta = 10;
            self.K_phi = 1;
            self.K_e=-1;
            self.alpha = deg2rad(20);
            self.home_D = 20;
            self.R = 20;
            self.Dt = Dt;

            self.state=1;
            self.yaw_corr = 0;
            self.output =[0;NaN;0];

            self.debug = [0;0];
        end
        
        function self = compute(self, x, y)
            
            % check if in "entering zone"
            % TODO:fix this
            if (abs(atan2(x(2), x(1)))<=self.alpha) && (sqrt(x(2)^2 + x(1)^2) > 15)
                rad2deg(atan2(x(2), x(1)));
                self.state=2;
            end
            
            % aproaching
            if self.state == 0
                % Not yet implemented
                0;

            % homing
            elseif self.state == 1
                x_home = self.home_D;
                y_home = 0;
                yaw_home = atan2(y_home-x(2),x_home-x(1));
                
                % obstacle avoidance with bearing measurement:
                bearing = y(5);
                distance = y(4);
                if (0<bearing) && (bearing<deg2rad(45)) && (distance<20)
                    phi = (deg2rad(45)-bearing)/deg2rad(45)*self.K_phi/distance;
                    self.yaw_corr = self.yaw_corr - phi*self.Dt;
                elseif (-deg2rad(45)<bearing) && (bearing<0) && (distance<20)  
                    phi = (bearing+deg2rad(45))/deg2rad(45)*self.K_phi/distance;
                    self.yaw_corr = self.yaw_corr + phi*self.Dt;
                elseif self.yaw_corr > 0
                    self.yaw_corr = max(0,self.yaw_corr - self.K_phi*self.Dt);
                elseif self.yaw_corr < 0
                    self.yaw_corr = min(0,self.yaw_corr + self.K_phi*self.Dt);
                end
                self.yaw_corr;
                yaw_d = wrapToPi(yaw_home+self.yaw_corr);
                self.output = [0.3; NaN; yaw_d];
                self.debug = [yaw_home, self.yaw_corr];
        
            % dock path following
            elseif self.state ==2
                % LOS path following 
                Delta = 2; %min(K_delta*exp(-cross_track), 5);
                x_los = x(1)-Delta;
                y_los = 0;

                yaw_des = atan2(y_los-x(2),x_los-x(1));

                % "publish references"
                self.output = real([0.3; NaN; yaw_des]);
            end
        end

      
    end
end