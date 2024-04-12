classdef EKF
    % filter

    properties
        X
        P1 
        P2
        P3
        P4
        Q1
        Q2
        Q3
        Q4
        P1_
        P2_
        P3_
        P4_
        x_pred
        
    end
    
    methods
        function obj = EKF(X0,P1_0,P2_0,P3_0,P4_0, P1,Q1,P2,Q2,P3,Q3, P4,Q4, Dt)
            % Initial 
            obj.X = X0;
            % inter filter covariances initialization
            obj.P1_ = P1_0;
            obj.P2_ = P2_0;
            obj.P3_ = P3_0;
            obj.P4_ = P4_0;

            % filter covariances
            obj.P1 = P1;
            obj.P2 = P2;
            obj.P3 = P3;
            obj.P4 = P4;
            obj.Q1 = Q1;
            obj.Q2 = Q2;
            obj.Q3 = Q3;
            obj.Q4 = Q4;

            
            % solve for linear kf for horizontal filter
            obj.K1 = dlqe(eye(4)+kron([0,Dt;0,0],eye(2)), eye(4), kron([1,0],eye(2)), Q1, R1);
            % solve for linear kf for rotational
            obj.K2 = dlqe(1,1,1, Q2, R2);
            % solve for linear kf for dock yaw
            obj.K4 = dlqe(1,1,1, Q4, R4);
            

        end
        
        function obj = predict(obj, input, Dt)
            obj.x_pred = zeros(8,1);
            %horizontal filter
            A = eye(4)+kron([0,Dt;0,0],eye(2));
            B = kron([Dt;0],eye(2));
            obj.x_pred(1:4) = A*obj.X(1:4)+B*input;
            obj.P1_ = A*obj.P1_*A' + obj.Q1;
            
            % rotational filter
            obj.x_pred(5) = obj.X(5);
            obj.P2_ = obj.P2_ + obj.Q2;

            % dock position estimate filter
            obj.x_pred(6:7) = obj.X(6:7);
            obj.P3_ = obj.P3_ + obj.Q3;
            
            % dock orientation estimate filter
            obj.x_pred(8) = obj.X(8);
            obj.P4_ = obj.P4_ + obj.Q4;       
        end

        function x_hat = update(obj, y)
            % horizontal filter
            obj.X(1:4) = obj.x_pred(1:4) + obj.K1*(y(1:2) - C*obj.x_pred(1:4));
            obj.P1_ = (eye(4) - K*C)*obj.P1_;
            
            % rotational filter
            obj.X(5) = obj.x_pred(5) + obj.K2*(y(5) - obj.x_pred(5));
            obj.P2_ = (1 - K2)*obj.P2_;

            % dock position estimate filter
            % TODO: finish implement this
            d  = 
            H = [obj.X]
H =  [eye(4),[0,0,0,0]']; 
    K = (P_nav_*H')*inv(H*P_nav_*H' + R_nav);
    e_nav(:,k) = y(1:4,k) - H*x_nav_;
    x_nav(:,k) = x_nav_ + K*e_nav(:,k);
             % dock orientation estimate filter
            x_hat(6) = obj.X(6);

            % current estimation filter
            % not active
            
        end
    end
end
