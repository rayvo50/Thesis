classdef ua_controller < handle
    properties
        %parameters:
        K1
        K2
        Ka
        K3
        K_phi
        alpha
        home_D
        R 
        Dt

        %variables:
        sigma
        crab_angle
        offset
        state
        yaw_corr
        output

        debug
    end
    
    methods
        function self = ua_controller(Dt)
            self.K1 = 2*0.7*0.2;
            self.K2 = 0.2^2;
            self.Ka= 1/Dt;
            self.K3 = 1.12;
            self.K_phi = 1;
            self.alpha = deg2rad(20);
            self.home_D = 20;
            self.R = 20;
            self.Dt = Dt;
            
            self.sigma =0;
            self.crab_angle =0;

            self.state=1;
            self.yaw_corr = 0;
            self.offset = 0;
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
                
                
                cte = -x(2);
                u = y(9);
      
                % sigma dynamics with anti-windup 
                sigma_dot = cte + self.Ka *( -self.K1/u * cte - self.K2/u * self.sigma - sigma_e( -self.K1/u * cte - self.K2/u * self.sigma));
                self.sigma = self.sigma + sigma_dot*self.Dt; 
                
                % u = -K1/U*e - K2/U*sigma 
                yaw_correction = -self.K1/u * cte - self.K2/u * self.sigma;
                  
                % psi_d = path_psi + asin(sat(u)) 
  
                self.crab_angle =  asin(sigma_e(yaw_correction));

                yaw_d = pi + self.crab_angle;
                
                % speed profile
                u_d = 0.3;
                u_d = 0.3 + 0.2*tanh(0.5*(x(1)-8));
                
                if x(1)<0.3
                    d_f = 2;
                    yaw_d = pi;% +self.crab_angle + self.crab_angle*(-cos(pi*x(1)/d_f) );
                end
                
                self.output = [u_d; NaN; yaw_d];
              
            end
        end

      
    end
end
