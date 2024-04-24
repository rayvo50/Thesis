classdef LOS_controller < handle
    properties
        %parameters:
        K_delta
        K_phi
        K_e
        alpha
        stage
        home_D
        R 

        %variables:
        state
        yaw_corr
        output
    end
    
    methods
        function self = LOS_Controller()
            self.K_delta = 10;
            self.K_phi = 1;
            self.K_e=-1;
            self.alpha = 20;
            self.home_D = 50;
            self.R = 20;

            self.stage_=1;
            self.yaw_corr = 0;
            self.output = [0,0];
        end
        
        function self = compute(self, x, y)

            % check if in "entering zone"
            if abs(atan2d(x(2)-x(7), x(1)-x(6)) - x(8))<=self.alpha
                self.stage=2;
            end
         
            % homing
            if self.stage == 1
                % predefined homing position
                x_home = x(6) + self.home_D*cosd(x(8));
                y_home = x(7) + self.home_D*sind(x(8));
                yaw_ = atan2d(y_home-x(2),x_home-x(1));
                
                % obstacle avoidance with bearing measurement:
                bearing = atan2d(y(5),y(4));
                if (0<bearing)&&(bearing<90)
                    phi = (90-bearing)/90*self.K_phi;
                    self.yaw_corr = self.yaw_corr - phi;
                elseif (-90<bearing)&&(bearing<0)
                    phi = (bearing+90)/90*self.K_phi;
                    self.yaw_corr = self.yaw_corr + phi;
                else
                    if self.yaw_corr > 0
                        self.yaw_corr = max(0, self.yaw_corr-self.K_phi*sign(self.yaw_corr)); 
                    elseif self.yaw_corr < 0
                        self.yaw_corr = min(0, self.yaw_corr-self.K_phi*sign(self.yaw_corr));
                    end
                end

                self.output = [0.3;yaw_des+self.yaw_corr]
                

            % dock path following
            elseif self.stage ==2
                m1 = tand(x(8));
                b1 = x(7) - m1*x(6);
               
                m2 = tand(x(8)+90);
                b2 = x(2) - m2*x(1);
               
                x_ = (b2-b1)/(m1-m2);
                y_ = m1*x_ +b1;
                
                cross_track = (m1*x(1)-x(2)+b1)/sqrt(m1^2+1);
                Delta = 2;%min(K_delta*exp(-cross_track), 5);
                x_los = x_ + Delta*cosd(x(8)+180);
                y_los = y_ + Delta*sind(x(8)+180);
                
                yaw_des = atan2d(y_los-x(2),x_los-x(1));% + Ke*cross_track;
            
                self.output = real([0.3;yaw_des]);
            end
        end

      
    end
end
