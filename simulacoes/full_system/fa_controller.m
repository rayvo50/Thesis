classdef fa_controller < handle
    properties
        %parameters:
        K_T
        K_x
        K_y
        K_phi
        
        K1
        K2
        Ka

        K_e
        alpha
        home_D
        R 
        Dt

        %variables:
        sigma
        yaw_corr
        state
        ksi
        output

        debug
        
    end
    
    methods
        function self = fa_controller(Dt)
            self.K_T = 0.2;
            self.K_y = 0.05;
            self.K_phi = 1;
            
            self.K1 = 2*0.7*0.2;
            self.K2 = 0.2^2;
            self.Ka= 1/Dt;

            self.K_e=-1;
            self.alpha = deg2rad(20);
            self.home_D = 20;
            self.R = 20;
            self.Dt = Dt;

            self.state=1;
            self.sigma= 0;
            self.yaw_corr = 0;
            self.ksi = 0;
            self.output =[0;NaN;0];

            self.debug = [0;0];
        end
        
        function self = compute(self, x, y)
            
            % check if in "entering zone"
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
                self.output = [0.3; 0; yaw_d];
        
            % dock path following
            elseif self.state ==2

                cte = -x(2);
                u = sqrt(y(9)^2+y(10)^2);
      
                % sigma dynamics with anti-windup 
                sigma_dot = cte + self.Ka *( -self.K1/u * cte - self.K2/u * self.sigma - sigma_e( -self.K1/u * cte - self.K2/u * self.sigma));
                self.sigma = self.sigma + sigma_dot*self.Dt; 
                
                % u = -K1/U*e - K2/U*sigma 
                yaw_correction = -self.K1/u * cte - self.K2/u * self.sigma;
                  
                % psi_d = path_psi + asin(sat(u)) 
                yaw_d = wrapTo2Pi(pi + asin(sigma_e(yaw_correction)));
                
                % speed profile
                V_d_ = 0.3 + 0.2*tanh(0.5*(x(1)-8));
                V_d = [V_d_*cos(yaw_d);V_d_*sin(yaw_d)];
                V_d_b = Rot(-x(5))*V_d;
                u_d = V_d_b(1);
                v_d = V_d_b(2);
                self.output = [V_d_; v_d; pi];



                % d = y(6)*cos(y(7)); 
                % p = y(6)*sin(y(7));
                % 
                % % qsi dynamics in DS frame
                % ksi_dot = self.K_T*( [min(0.6, -(0.3 +0.2*tanh(2*(x(1)-2))) *(1-(1-exp(-2*abs(p)))^2)) ;-self.K_y*p] - Rot(x(5))*y(9:10)); 
                % self.ksi = self.ksi + ksi_dot*self.Dt;
                % self.debug = Rot(-x(5))*[min(0.6, -(0.3 +0.2*tanh(2*(x(1)-2))) *d*(1-(1-exp(-2*abs(p)))^2)) ;-self.K_y*p];
                % 
                % % rotate velocity reference to bofy frame
                % v_ref = Rot(-x(5))*self.ksi;
                % 
                % % "publish references"
                % self.output = [v_ref(1); v_ref(2); pi];
            end
        end

      
    end
end
