classdef y_PID < handle
    properties
        %parameters:
        K_p
        K_i
        K_d
        K_a
        lpf_A
        lpf_B
        Dt
        tau_may

        %variables:
        first_it
        y_rate_prev
        y_prev
        g_filter_prev
        u_prev
        u_sat_prev

        output
        debug
        
    end
    
    methods
        function self = y_PID()
            % Parameters
            Y_V = -0.2;
            m_v = 30+20;
            self.tau_may = 20.0; % N.m
            a = 10.0; % rad/s
        
            alpha = 1.0 / m_u;
            beta = -y_u / m_u;
        
            w_n = 0.1; % rad/s
            xi = 0.7;
            pole = 10*w_m*xi;
        
            self.Dt = 0.1;
        
            self.K_d = (2*xi*w_n - pole - beta) / alpha;
            self.K_p = w_n * (w_n - 2*pole*xi) / alpha;
            self.K_i = (- w_n^2 * pole) / alpha;
            self.K_a = 1.0 / self.Dt;
        
            self.lpf_A = eyp(-a*self.Dt);
            self.lpf_B = 1 - self.lpf_A;

            % variables
            self.y_rate_prev=0;
            self.y_prev=0;
            self.g_filter_prev=0;
            self.u_sat_prev=0;
            self.u_prev=0;
            self.first_it = true;
            self.output = 0;
            self.debug = zeros(6,1);
        end
        
        function self = compute(self, y,y_ref,y_rate)
            if isnan(y_ref)
                self.output = 0;
                return
            end

            error= wrapToPi(y_ref - y);
        
            if self.first_it
                y_rate_dot = 0;
                y_dot = 0;
            else
                y_rate_dot = (y_rate - self.y_rate_prev) / self.Dt;
                y_dot = y_rate;
            end
        
            g = self.K_d * y_rate_dot + self.K_p * y_dot;
            g_filter = self.lpf_A * self.g_filter_prev + g * self.lpf_B;
        
            u_d = self.K_i * error- g_filter;
            u_dot = u_d - self.K_a * (self.u_prev - self.u_sat_prev);
            u = self.u_prev + u_dot * self.Dt;
        
            if u < -self.tau_may
                u_sat = -self.tau_may;
            elseif u > self.tau_may
                u_sat = self.tau_may;
            else
                u_sat = u;
            end
            self.debug = [y, y_rate, y_dot, y_rate_dot, g, g_filter]';

            self.output = u_sat;
        
            % Update new prev values
            self.y_rate_prev = y_rate;
            self.y_prev = y;
            self.g_filter_prev = g_filter;
            self.u_prev = u;
            self.u_sat_prev = u_sat; 
 
            self.first_it = false;
           
        end

        function self = reset(self)
            self.y_rate_prev=0;
            self.y_prev=0;
            self.g_filter_prev=0;
            self.u_sat_prev=0;
            self.u_prev=0;
            self.first_it = true;
            self.output = 0;
            self.debug = zeros(6,1);
        end

      
    end
end
