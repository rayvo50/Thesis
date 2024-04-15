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
        function obj = EKF(X0, Dt)
            % Initial 
            obj.X = X0;
            % internal filter covariances initialization
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
            obj.P1_ = (eye(4) - obj.K1*C)*obj.P1_;
            
            % rotational filter
            obj.X(5) = obj.x_pred(5) + obj.K2*(y(5) - obj.x_pred(5));
            obj.P2_ = (1 - obj.K2)*obj.P2_;

            % dock position estimate filter
            d  = sqrt((obj.X(1)-obj.X(7))^2 + (obj.X(2)-obj.X(8))^2 );
            H = [(obj.X(6)-obj.X(1))/d, (obj.X(7)-obj.X(1))/d;
                (obj.X(2)-obj.X(7))/d^2, (obj.X(6)-obj.X(1))/d^2];
            
            K3 = (obj.P3_*H')*inv(H*obj.P3*H' + obj.Q3);
            y_ = [sqrt(y(6)^2+y(7)^2); atan2d(y(7),y(6))];
            hx_ = [sqrt( (obj.X(1)-obj.X(6))^2 + (obj.X(2)-obj.X(7))^2 ); atan2d((obj.X(7)-obj.X(2)), (obj.X(6)-obj.X(1))) - obj.X(8)];
            obj.X(6:7) = obj.x_pred(6:7) + K3*(y_-hx_);
            obj.P3_ = (eye(2) - K3*H)*obj.P3_;
            
            % dock orientation estimate filter
            r1 = -dot(y(8:9),y(6:7));
            r2 = [0,0,1]*cross([y(8:9);0],[y(6:7);0]);
            y_ = atan2d(r2,r1) + obj.X(5);
            obj.X(8) = obj.x_pred(8) + obj.K4*(y_ - obj.x_pred(8));
            obj.P4_ = (1 - obj.K4)*obj.P4_;
             
        end
    end
end
