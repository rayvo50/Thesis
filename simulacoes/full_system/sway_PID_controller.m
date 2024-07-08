classdef sway_PID_controller < handle
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
        sway_rate_prev
        sway_prev
        g_filter_prev
        u_prev
        u_sat_prev
        output
        
    end
    
    methods
        function self = sway_PID_controller()
            % Parameters
            Y_v = -50;
            m_v = 30+30;
            self.tau_max = 25.0; % N.m
            a = 1.0; % rad/s

            w_n = 1; % rad/s
            qsi = 0.6;
        
            % self.Dt calculation
            self.Dt = 0.1;

            self.K_a = 1.0 / self.Dt;
            self.K_p = 2*qsi*w_n*m_v + Y_v;
            self.K_i = w_n^2*m_v;
        
            self.lpf_A = exp(-a*self.Dt);
            self.lpf_B = 1 - self.lpf_A;

            % variables
            self.sway_rate_prev = 0;
            self.sway_prev= 0;
            self.g_filter_prev=0;
            self.u_sat_prev=0;
            self.u_prev=0;
            self.first_it = true;
            self.output = 0;
        end
        
        function self = compute(self, sway, sway_ref)
            if isnan(sway_ref)
                self.output = 0;
                return
            end
            
            error = sway_ref - sway;
            
            sway_rate = (sway - self.sway_prev)/self.Dt;
            if self.first_it
                sway_dot = 0;
            else
                sway_dot = wrapToPi(sway - self.sway_prev) / self.Dt;
            end
        
            g = self.K_p * sway_dot;
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
            self.sway_rate_prev = sway_rate;
            self.sway_prev = sway;
            self.g_filter_prev = g_filter;
            self.u_prev = u;
            self.u_sat_prev = u_sat; 
 
            self.first_it = false;
           
        end

      
    end
end
