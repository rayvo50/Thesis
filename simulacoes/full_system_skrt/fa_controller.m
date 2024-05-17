classdef fa_controller < handle
    properties
        %parameters
        alpha
        home_D
        Dt
        K_phi
        
        K_T 
        K_x
        K_y

        %variables:
        state
        yaw_corr
        ksi

        output

        debug
        
    end
    
    methods
        function self = fa_controller(Dt)
            self.K_T = 1;
            self.K_x = 1;
            self.K_y = 1;
            self.K_phi = 0.1;
            self.alpha = deg2rad(10);
            self.home_D = 20;
            self.Dt = Dt;

            self.state=1;
            self.yaw_corr = 0;
            self.ksi = 0;
            self.output =[0;0;NaN];

            self.debug = [0;0];
        end
        
        function self = compute(self, x, y)
            
            % check if in "entering zone"
            if (abs(atan2(x(2)-x(7), x(1)-x(6)) - x(8))<=self.alpha) && (sqrt( (x(2)-x(7))^2 + (x(1)-x(6))^2 ) > 15)
                self.state=2;
            end
            
            % aproaching
            if self.state == 0
                0;

            % homing
            elseif self.state == 1
                x_home = x(6) + self.home_D*cos(x(8));
                y_home = x(7) + self.home_D*sin(x(8));
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
                    self.yaw_corr = max(0,self.yaw_corr -2*self.K_phi*self.Dt);
                elseif self.yaw_corr < 0
                    self.yaw_corr = min(0,self.yaw_corr +2*self.K_phi*self.Dt);
                end
                yaw_d = wrapToPi(yaw_home+self.yaw_corr);
                self.output = [0.3;0;yaw_d];
                self.debug = [yaw_home, self.yaw_corr];
        
            % dock path following
            elseif self.state ==2
                d = y(4)*cos(y(5)); 
                p = y(4)*sin(y(5));

                % qsi dynamics in DS frame
                ksi_dot = self.K_T*([self.K_x*d;self.K_y*p] - Rot(-x(11))*y(9:10)); 
                self.ksi = self.ksi + ksi_dot*self.Dt;

                % rotate velocity reference to bofy frame
                v_ref = Rot(x(11))*self.ksi;

                % "publish references"
                self.output = [v_ref(1); v_ref(2); pi];
            end
        end

      
    end
end
