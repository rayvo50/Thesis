classdef y_SMC < handle
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
        
        function self = y_SMC()

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

        function k_ = get_k(self, x, xd, yaw, u, v, r)
            mu = 50;
            mv = 60; 
            Xu = 0.2; 
            Xuu = 25;
            Yv = 55.1;
            Yvv = 101.3;
            Beta = 1.2;
            delta = 0.2;
            F = delta*(r*(cos(yaw)*u -sin(yaw)*v)) + delta*sin(yaw)/mu*(-Xu*u-Xuu*u*abs(u)+mv*v*r) + delta*cos(yaw)/mv*(-Yv*v-Yvv*v*abs(v)+mu*u*r);
            %F = delta*Zw/mw*x(2) + delta*Zww/mw*x(2)*abs(x(2)) + sign(xd(2))*delta*gw/mw;
            k_ = Beta*self.eta + F + (Beta-1)*abs(xd(3) - self.lambda*(x(2)-xd(2)));
        end   

        function self = compute(self, y, y_ref, y_rate_ref, y_rate_rate_ref, yaw, u, v, r, vcy)
            x = [y, -u*sin(yaw)+v*cos(yaw)+vcy];
            xd = [y_ref, y_rate_ref, y_rate_rate_ref];
            mu = 50;
            mv = 60; 
            Xu = 0.2; 
            Xuu = 25;
            Yv = 55.1;
            Yvv = 101.3;
            Beta = 1.2;
            self.e=wrapToPi(x(1)-xd(1));


            if isnan(y_ref)
                self.output=0;
                return;
            end
            
            % Equivalent Control:
            % û = - f(x) + x_d'' - lambda * x~'
            u_eq = -( r*(cos(yaw)*u -sin(yaw)*v) + sin(yaw)/mu*(-Xu*u-Xuu*u*abs(u)+mv*v*r) + cos(yaw)/mv*(-Yv*v-Yvv*v*abs(v)+mu*u*r) ) +xd(3) - self.lambda*(x(2)-xd(2));


            % Switching control: 
            % Compute Phi_dot
            Phi_dot = -self.lambda*self.Phi + get_k(self, xd, xd, yaw, u, v, r);
            % integrate to obtain Phi
            self.Phi = self.Phi + self.Dt*Phi_dot;
            
            % compute s
            self.s = x(2)-xd(2) + self.lambda*wrapToPi(x(1)-xd(1));
            % compute switching gain
            k = get_k(self, x, xd, yaw, u, v, r) - get_k(self, xd, xd, yaw, u, v, r) + self.lambda*self.Phi/Beta;
            % switching law
            u_s= -k*sat(self.s/self.Phi);
            
            % Final control law:
            % u = b^⁻¹ [ u_eq  + u_s]
            self.output = (1/(sin(yaw)^2/mu + cos(yaw)^2/mv)) * (u_s);
            self.debug=0;
            
        end

      
    end
end







 

