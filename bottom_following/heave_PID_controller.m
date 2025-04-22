classdef heave_PID_controller < handle
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
        heave_rate_prev
        heave_prev
        g_filter_prev
        u_prev
        u_sat_prev
        output
        
    end
    
    methods
        function self = heave_PID_controller()
            % Parameters
            m = 11.2;
            Xudot = 29.9081;
            m_u = m+Xudot;
            X_u = -1.1130; 
           
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
            self.heave_rate_prev = 0;
            self.heave_prev= 0;
            self.g_filter_prev=0;
            self.u_sat_prev=0;
            self.u_prev=0;
            self.first_it = true;
            self.output = 0;
        end
        
        function self = compute(self, heave, heave_ref)
            if isnan(heave_ref)
                self.output = 0;
                return
            end
            error = heave_ref - heave;
            
            heave_rate = (heave - self.heave_prev)/self.Dt;
            if self.first_it
                heave_dot = 0;
            else
                heave_dot = (heave - self.heave_prev) / self.Dt;
            end
        
            g = self.K_p * heave_dot;
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
            self.heave_rate_prev = heave_rate;
            self.heave_prev = heave;
            self.g_filter_prev = g_filter;
            self.u_prev = u;
            self.u_sat_prev = u_sat; 
 
            self.first_it = false;
           
        end

      
    end
end
