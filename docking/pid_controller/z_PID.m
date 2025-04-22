classdef z_PID < handle
    properties
        %parameters:
        K_p
        K_i
        K_d
        K_a
        lpf_A
        lpf_B
        Dt
        tau_max

        %variables:
        first_it
        z_rate_prev
        z_prev
        g_filter_prev
        u_prev
        u_sat_prev

        tau
        debug
        
    end
    
    methods
        function self = z_PID()
            % Parameters
            Z_w = -4.1879;
            m_w = 30+70;
            self.tau_max = 20.0; % N.m
            a = 10.0; % rad/s
        
            alpha = 1.0 / m_w;
            beta = -Z_w / m_w;
        
            w_n = 0.2; % rad/s
            xi = 0.7;
            pole = -10*xi*w_n;
        
            self.Dt = 0.1;
        
            self.K_d = (2*xi*w_n - pole - beta) / alpha;
            self.K_p = w_n * (w_n - 2*pole*xi) / alpha;
            self.K_i = (- w_n^2 * pole) / alpha;
            self.K_a = 1.0 / self.Dt;
        
            self.lpf_A = exp(-a*self.Dt);
            self.lpf_B = 1 - self.lpf_A;

            % variables
            self.z_rate_prev=0;
            self.z_prev=0;
            self.g_filter_prev=0;
            self.u_sat_prev=0;
            self.u_prev=0;
            self.first_it = true;
            self.tau = 0;
            self.debug = zeros(6,1);
        end
        
        function self = compute(self, z,z_ref,z_rate)
            if isnan(z_ref)
                self.tau = 0;
                return
            end

            error= wrapToPi(z_ref - z);
        
            if self.first_it
                z_rate_dot = 0;
                z_dot = 0;
            else
                z_rate_dot = (z_rate - self.z_rate_prev) / self.Dt;
                z_dot = z_rate;%(z - self.z_prev) / self.Dt;
            end
        
            g = self.K_d * z_rate_dot + self.K_p * z_dot;
            g_filter = self.lpf_A * self.g_filter_prev + g * self.lpf_B;
        
            u_d = self.K_i * error- g_filter;
            u_dot = u_d - self.K_a * (self.u_prev - self.u_sat_prev);
            u = self.u_prev + u_dot * self.Dt;
        
            if u < -self.tau_max
                u_sat = -self.tau_max;
            elseif u > self.tau_max
                u_sat = self.tau_max;
            else
                u_sat = u;
            end
            self.debug = [z, z_rate, z_dot, z_rate_dot, g, g_filter]';

            self.tau = u_sat;
        
            % Update new prev values
            self.z_rate_prev = z_rate;
            self.z_prev = z;
            self.g_filter_prev = g_filter;
            self.u_prev = u;
            self.u_sat_prev = u_sat; 
 
            self.first_it = false;
           
        end

        function self = reset(self)
            self.z_rate_prev=0;
            self.z_prev=0;
            self.g_filter_prev=0;
            self.u_sat_prev=0;
            self.u_prev=0;
            self.first_it = true;
            self.tau = 0;
            self.debug = zeros(6,1);
        end

      
    end
end
