classdef yaw_SMC < handle
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
        
        function self = yaw_SMC()

            % Parameters
            self.lambda = 0.9;
            self.eta = 10.0;          

            % Initialize boundary layer width
            Beta = 1.1;
            self.Phi = Beta*self.eta/self.lambda;
            
            self.output = 0;
            self.s = nan;
            self.Dt=0.1;
            self.v=0;
           
        end

        function k_ = get_k(self, x, xd, V)
            mu = 30+20;
            mv = 30+30; 
            mr = (4.14 +0.5);
            muv = mu-mv; 
            Nr = 4.14;
            Nrr = 6.23;
            Beta = 1.1;
            delta = 0.1;
            F = delta*Nr/mr*x(2) + delta*Nrr/mr*x(2)*abs(x(2)) + delta*muv/mr*V(1)*V(2);
            k_ = Beta*self.eta + F + (Beta-1)*abs(xd(3) - self.lambda*(x(2)-xd(2)));
        end   

        
        function self = compute(self, yaw, yaw_rate, yaw_ref, yaw_rate_ref, yaw_rate_rate_ref, u, v)
            x = [yaw, yaw_rate];
            xd = [yaw_ref, yaw_rate_ref, yaw_rate_rate_ref];
            V = [u,v];
            mu = 30+20;
            mv = 30+30; 
            mr = 4.14 +0.5;
            muv = mu-mv; 
            Nr = -4.14;
            Nrr = -6.23;
            Beta = 1.1;
            self.e=wrapToPi(x(1)-xd(1));


            if isnan(yaw_ref)
                self.output=0;
                return;
            end
            
            % Equivalent Control:
            % û = - f(x) + x_d'' - lambda * x~'
            u_eq = -(Nr/mr*x(2) + Nrr/mr*x(2)*abs(x(2)) + muv/mr*u*v) +xd(3) - self.lambda*(x(2)-xd(2));


            % Switching control: 
            % Compute Phi_dot
            Phi_dot = -self.lambda*self.Phi + get_k(self, xd, xd, V);
            % integrate to obtain Phi
            self.Phi = self.Phi + self.Dt*Phi_dot;
            
            % compute s
            self.s = x(2)-xd(2) + self.lambda*wrapToPi(x(1)-xd(1));
            % compute switching gain
            k = get_k(self, x, xd, V) - get_k(self, xd, xd, V) + self.lambda*self.Phi/Beta;
            % switching law
            u_s= -k*sat(self.s/self.Phi);
            
            % Final control law:
            % u = b^⁻¹ [ u_eq  + u_s]
            self.output = mr * (u_eq+u_s);
            self.debug=0;
            
        end

      
    end
end


%% MERDA
% self.e = wrapToPi(yaw - yaw_ref);
%             self.s = yaw_rate-yaw_rate_ref + self.lambda*self.e;
%             %reaching_law = -self.epsilon*(abs(self.s)^0.5)*sign(self.s) -self.k*self.s;
%             k1 = 1;
%             k2=0.1;
%             self.v = self.v+k2*sign(self.s)*self.Dt;
%             reaching_law = -k1*sqrt(abs(self.s))*sigmoidmoid(self.s) - self.v;
% 
%             u_eq = - muv*u*v + Nr*yaw_rate +Nrr*yaw_rate*abs(yaw_rate);
%             self.output = mr*(reaching_law - self.lambda*(yaw_rate-yaw_rate_ref) + yaw_rate_rate_ref) + u_eq;