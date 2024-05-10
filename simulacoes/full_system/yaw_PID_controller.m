classdef yaw_PID_controller < handle
    properties
        %parameters:
        K_p
        K_i
        K_r
        K_a
        lpf_A
        lpf_B
        Dt
        tau_max

        %variables:
        first_it
        yaw_rate_prev
        yaw_prev
        g_filter_prev
        u_prev
        u_sat_prev
        output
        
    end
    
    methods
        function self = yaw_PID_controller()
            % Parameters
            N_r = -0.5;
            I_z = 0.14;
            self.tau_max = 10.0; % N.m
            a = 10.0; % rad/s
        
            alpha = 1.0 / I_z;
            beta = -N_r / I_z;
        
            w_n = 10; % rad/s
            qsi = 0.9;
            pole = -4;
        
            % self.Dt calculation
            self.Dt = 0.1;%1.0 / frequency;
        
            self.K_r = (2*qsi*w_n - pole - beta) / alpha;
            self.K_p = w_n * (w_n - 2*pole*qsi) / alpha;
            self.K_i = (- w_n^2 * pole) / alpha;
            self.K_a = 1.0 / self.Dt;
        
            self.lpf_A = exp(-a*self.Dt);
            self.lpf_B = 1 - self.lpf_A;

            % variables
            self.yaw_rate_prev=0;
            self.yaw_prev=0;
            self.g_filter_prev=0;
            self.u_sat_prev=0;
            self.u_prev=0;
            self.first_it = true;
            self.output = 0;
        end
        
        function self = compute(self, yaw,yaw_ref,yaw_rate)
            error = deg2rad(wrapTo180(yaw_ref - yaw));
        
            if self.first_it
                yaw_rate_dot = 0;
                yaw_dot = 0;
            else
                yaw_rate_dot = (yaw_rate - self.yaw_rate_prev) / self.Dt;
                yaw_dot = wrapToPi(yaw - self.yaw_prev) / self.Dt;
            end
        
            g = self.K_r * yaw_rate_dot + self.K_p * yaw_dot;
            g_filter = self.lpf_A * self.g_filter_prev + g * self.lpf_B;
        
            u_d = self.K_i * error - g_filter;
            u_dot = u_d - self.K_a * (self.u_prev - self.u_sat_prev);
            u = self.u_prev + u_dot * self.Dt;
        
            if u < -self.tau_max
                u_sat = -self.tau_max;
            elseif u > self.tau_max
                u_sat = self.tau_max;
            else
                u_sat = u;
            end

            self.output = u_sat;
        
            % Update new prev values
            self.yaw_rate_prev = yaw_rate;
            self.yaw_prev = yaw;
            self.g_filter_prev = g_filter;
            self.u_prev = u;
            self.u_sat_prev = u_sat; 
 
            self.first_it = false;
           
        end

      
    end
end
