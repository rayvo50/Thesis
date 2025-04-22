classdef surge_PID_controller < handle
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
        surge_rate_prev
        surge_prev
        g_filter_prev
        u_prev
        u_sat_prev
        output
        
    end
    
    methods
        function self = surge_PID_controller()
            % Parameters
            m = 11.2;
            Xudot = 27.08;
            m_u = m+Xudot;
            X_u = -0.1213 ; 
           
            self.tau_max = 25.0; % N.m
            a = 10.0; % rad/s
            w_n = 5; % rad/s
            qsi = 0.8;
        
            self.Dt = 0.1;
        
            self.K_a = 1.0 / self.Dt;
            self.K_p = 2*qsi*w_n*m_u + X_u;
            self.K_i = w_n^2*m_u;
        
            self.lpf_A = exp(-a*self.Dt);
            self.lpf_B = 1 - self.lpf_A;

            % variables
            self.surge_rate_prev = 0;
            self.surge_prev= 0;
            self.g_filter_prev=0;
            self.u_sat_prev=0;
            self.u_prev=0;
            self.first_it = true;
            self.output = 0;
        end
        
        function self = compute(self, surge, surge_ref)
            if isnan(surge_ref)
                self.output = 0;
                return
            end
            error = surge_ref - surge;
            
            surge_rate = (surge - self.surge_prev)/self.Dt;
            if self.first_it
                surge_dot = 0;
            else
                surge_dot = (surge - self.surge_prev) / self.Dt;
            end
        
            g = self.K_p * surge_dot;
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
            self.surge_rate_prev = surge_rate;
            self.surge_prev = surge;
            self.g_filter_prev = g_filter;
            self.u_prev = u;
            self.u_sat_prev = u_sat; 
 
            self.first_it = false;
           
        end

      
    end
end
