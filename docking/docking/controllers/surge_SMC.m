classdef surge_SMC < handle
    properties
        epsilon;
        k;
        lambda;
        tau;
        s;
        output;
        u1;
        Dt;
        e;
    end
    
    methods
        function self = surge_SMC()
            self.epsilon = 1;
            self.k = 1;
            self.lambda = 1;
            self.tau = 0;
            self.output=0;
            self.u1 =0;
            self.Dt=0.1;
            self.e=0;
            self.s=0;
        end
        
        function self = compute(self, u, u_ref, u_ref_dot);
            mu = 30+20;
            Xu = 0.2;
            Xuu = 25;

            self.e = u - u_ref;
            self.s = self.e;
            
            U_M=20;
            alpha=1;
            k1=10;
            if abs(self.output)>U_M
                u1_dot = -self.output;
            else
                u1_dot = -alpha*sign(self.s);
            end 
            self.u1 = self.u1 + self.Dt*u1_dot;
            self.output = -k1*sqrt(abs(self.s))*sigmoid(self.s) + self.u1;
            
        end

      
    end
end
