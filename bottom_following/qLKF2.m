classdef qLKF2 < handle
    % This implements a estimator to obtain an estimate of the vector D.
    % See the notes for reference. This is the version where 
    %       xhat = [D;D_dot].
    % This is similar to a "sensor fusion" kalman filter 

    properties
        xhat    % Estimator State
        D_dot   % Placeholder because in the other estimator D_dot is not part of the state
        P       % Filter covariance
        R       % Measurement Noise Covariance
        Q       % Process Noise Covariance
           
        alpha   % angle of the second echosounder
        Dt      % Timestep
        debug   % handy variable for debugging if needed
        
    end
    
    methods
        % Constructor
        function self = qLKF2(y0, Dt, alpha)
            self.Dt = Dt;
            self.alpha = alpha;
            disp(self.alpha)
            
            % This is not needed if the state is initalized with [0;0]
            % compute D from initial measurement 
            h1 = [0;y0(1)];
            h2 = Rot(-self.alpha)*[0;y0(2)];
            S = h2 - h1;
            Ps = eye(2) - (S * S')/(norm(S)^2);
            D = Ps * h1;
            
            % initialize filter state and covariance
            self.xhat =  [D;0;0];%D;
            self.P = eye(4);%0.1*norm(D)*eye(2);
 
            % horizontal filter parameters
            self.R = 10*eye(4);
            self.Q = 0.01*eye(4);

            self.debug = zeros(1,1);
        end
        
        function self = compute(self, u, y)
        
            % --- Predict Step ---
            Ak =  kron([1,self.Dt;0,1], eye(2));
            self.xhat = Ak*self.xhat;
            self.P = Ak*self.P*Ak' + self.Q;

            % --- Update Step ---
            % Compute D from the measurements h1 and h2
            h1 = [0;y(1)];
            h2 = Rot(-self.alpha)*[0;y(2)];
            S = h2 - h1;
            Ps = eye(2) - (S * S')/(norm(S)^2);
            D = Ps*h1;

            % Compute D_dot from previous projection matrix and V_dvl
            S1 = [self.xhat(2); -self.xhat(1)];
            S1 = S1/norm(S1);
            Ps1 = eye(2) - (S1 * S1')/(norm(S1)^2);
            Ddot = -Ps1*u;
            
            % build measurement vector
            y = [D;Ddot];
            
            % do the actual update step computation
            K = self.P*inv(self.P + self.R);
            self.xhat = self.xhat + K * (y - self.xhat);
            self.P = (eye(4)-K)*self.P;

            % this is just to output D_dot using the same variable as the
            % other filter's implementation
            self.D_dot = self.xhat(3:4);

        end
    end
end
