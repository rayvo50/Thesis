classdef LOS_Controller < handle
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
        
    end
    
    methods
        function self = LOS_Controller(Dt)
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
            
            % check if in "entering zone"
            if abs(atan2d(x(2)-x(7), x(1)-x(6)) - x(8))<=self.alpha
                self.state=2;
            end
         
            % homing
            if self.state == 1
                x_home = x(6) + self.home_D*cosd(x(8));
                y_home = x(7) + self.home_D*sind(x(8));
                yaw_home = atan2d(y_home-x(2),x_home-x(1));
                
                % obstacle avoidance with bearing measurement:
                bearing = atan2d(y(5),y(4));
                distance = sqrt(y(5)^2+y(4)^2);
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
                self.output = [0.3;yaw_d];
        
            % dock path following
            elseif self.state ==2
                m1 = tand(x(8));
                b1 = x(7) - m1*x(6);
               
                m2 = tand(x(8)+90);
                b2 = x(2) - m2*x(1);
               
                x_ = (b2-b1)/(m1-m2);
                y_ = m1*x_ +b1;
                
                cross_track = (m1*x(1)-x(2)+b1)/sqrt(m1^2+1);
                Delta = 3; %min(K_delta*exp(-cross_track), 5);
                x_los = x_ + Delta*cosd(x(8)+180);
                y_los = y_ + Delta*sind(x(8)+180);
                
                yaw_des = atan2d(y_los-x(2),x_los-x(1)); % + Ke*cross_track;
            
                self.output = real([0.3;yaw_des;NaN]);
            end
        end

      
    end
end
