classdef LOS_Controller_r < handle
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
        time
        output
        
    end
    
    methods
        function self = LOS_Controller_r(Dt)
            self.K_delta = 10;
            self.K_phi = 1;
            self.K_e=-1;
            self.alpha = 10;
            self.home_D = 20;
            self.R = 20;
            self.Dt = Dt;

            self.state=0;
            self.yaw_corr = 0;
            self.time = 0;
            self.output =[0;0;NaN];
        end
        
        function self = compute(self, x, y, dock_pos_cov, dock_yaw_cov)
            
            
            % searching phase -> aproaching phase
            if self.time > 700 && self.state==0
                self.state=1;
            end
            % aproaching phase -> homing phase
            if ~isnan(x(8)) && self.state==1
                self.state=2;
            end    
            % homing phase -> docking phase
            if (abs(atan2d(x(2)-x(7), x(1)-x(6)) - x(8))<=self.alpha) && (sqrt( (x(2)-x(7))^2 + (x(1)-x(6))^2 ) > 15) && self.state==2
                self.state=3;
            end
            
            % searching
            if self.state == 0
                yaw_dock = atan2d(x(7)-x(2),x(6)-x(1));
                yaw_d = yaw_dock + 130*sin(0.05*self.time);
                self.time = self.time+self.Dt;
                self.output = [0.6;yaw_d;NaN];

            % aproaching
            elseif self.state == 1
                yaw_d = atan2d(x(7)-x(2),x(6)-x(1));
                self.output = [0.3;yaw_d;NaN];

            % homing
            elseif self.state == 2
                x_home = x(6) + self.home_D*cosd(x(8));
                y_home = x(7) + self.home_D*sind(x(8));
                yaw_home = atan2d(y_home-x(2),x_home-x(1));
                
                % obstacle avoidance with bearing measurement:
                bearing = y(5);
                distance = y(4);
                if (0<bearing) && (bearing<45) && (distance<20)
                    phi = (45-bearing)/45*self.K_phi/distance*10;
                    self.yaw_corr = self.yaw_corr - phi*self.Dt;
                elseif (-45<bearing) && (bearing<0) && (distance<20)  
                    phi = (bearing+45)/45*self.K_phi/distance*10;
                    self.yaw_corr = self.yaw_corr + phi*self.Dt;
                elseif self.yaw_corr > 0
                    self.yaw_corr = max(0,self.yaw_corr -2*self.K_phi*self.Dt);
                elseif self.yaw_corr < 0
                    self.yaw_corr = min(0,self.yaw_corr +2*self.K_phi*self.Dt);
                end
                yaw_d = wrapTo180(yaw_home+self.yaw_corr);
                self.output = [0.3;yaw_d;NaN];
        
            % dock path following
            elseif self.state ==3
                Delta = 5; %min(K_delta*exp(-cross_track), 5);
                x_los = x(9)-Delta;
                y_los = 0;

                yaw_des = atan2d(y_los-x(10),x_los-x(9));
            
                self.output = real([0.3;NaN;yaw_des]);
            end
        end

      
    end
end