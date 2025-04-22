classdef z_SMC < handle
    properties
        lambda;
        eta;
        Phi;
        output;
        s;
        e;
        u;
        Dt;
        v;
        debug;
        
    end
    
    methods
        
        function self = z_SMC()

            % Parameters
            self.lambda =0.5;
            self.eta = 10.0;          

            % Initialize boundary layer width
            Beta = 1.1;
            self.Phi = Beta*self.eta/self.lambda;
            
            self.output = 0;
            self.s = nan;
            self.Dt=0.1;
            self.v=0;
           
        end

        function k_ = get_k(self, x, xd)
            mw = 100;
            Zw = 4.1879;
            Zww = 40.9649;
            gw = 3.5;
            Beta = 1.1;
            delta = 0.2;
            F = delta*Zw/mw*x(2) + delta*Zww/mw*x(2)*abs(x(2)) + sign(xd(2))*delta*gw/mw;
            k_ = Beta*self.eta + F + (Beta-1)*abs(xd(3) - self.lambda*(x(2)-xd(2)));
        end   

        
        function self = compute(self, z, z_rate, z_ref, z_rate_ref, z_rate_rate_ref)
            x = [z, z_rate];
            xd = [z_ref, z_rate_ref, z_rate_rate_ref];
            mw = 30+70;
            Zw = -4.1879;
            Zww = -40.9649;
            gw = 3.5;
            Beta = 1.1;
            self.e=wrapToPi(x(1)-xd(1));


            if isnan(z_ref)
                self.output=0;
                return;
            end
            
            % Equivalent Control:
            % û = - f(x) + x_d'' - lambda * x~'
            u_eq = -(Zw/mw*x(2) + Zww/mw*x(2)*abs(x(2)) - gw/mw) +xd(3) - self.lambda*(x(2)-xd(2));


            % Switching control: 
            % Compute Phi_dot
            Phi_dot = -self.lambda*self.Phi + get_k(self, xd, xd);
            % integrate to obtain Phi
            self.Phi = self.Phi + self.Dt*Phi_dot;
            
            % compute s
            self.s = x(2)-xd(2) + self.lambda*wrapToPi(x(1)-xd(1));
            % compute switching gain
            k = get_k(self, x, xd) - get_k(self, xd, xd) + self.lambda*self.Phi/Beta;
            % switching law
            u_s= -k*sat(self.s/self.Phi);
            
            % Final control law:
            % u = b^⁻¹ [ u_eq  + u_s]
            self.output = mw * (u_eq+u_s);
            self.debug=0;
            
        end

      
    end
end