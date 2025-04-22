classdef qLKF < handle
    properties
        xhat
        D_dot
        P
        R
        Q
           
        alpha
        Dt
        debug
        
    end
    
    methods
        function self = qLKF(y0, Dt, alpha)
            self.Dt = Dt;
            self.alpha = alpha;
            % compute D from initial measurement
            h1 = [0;y0(1)];
            h2 = Rot(-alpha)*[0;y0(2)];
            S = h2 - h1;
            Ps = eye(2) - (S * S')/(norm(S)^2);
            D = Ps * h1;
            
            % initialize filter state
            self.xhat = D;
            self.P = eye(2);%0.1*norm(D)*eye(2);
 
            % horizontal filter
            self.R = 100*eye(2);%0.2*eye(2);    % 
            self.Q = eye(2);%diag(0.1*Dt);

            self.debug = zeros(1,1);
        end
        
        function self = compute(self, u, y)
            
            % obtain the measurements in the correct shape and compute 
            % the projection matrix 
            h1 = [0;y(1)];
            h2 = Rot(-self.alpha)*[0;y(2)];
            S = h2 - h1;
            Ps = eye(2) - (S * S')/(norm(S)^2);
            D = Ps*h2;

            % Regular kalman filter equations
            % Predict 
            S1 = [self.xhat(2); -self.xhat(1)];
            S1 = S1/norm(S1);
            Ps1 = eye(2) - (S1 * S1')/(norm(S1)^2);
            self.D_dot = -Ps1*u;
            self.xhat = self.xhat + self.Dt* self.D_dot;
            self.P = self.P + self.Q;

            
            % Update
            K = self.P*inv(self.P + self.R);
            self.xhat = self.xhat + K * (D - self.xhat);
            self.P = (eye(2)-K)*self.P;

        end
    end
end
