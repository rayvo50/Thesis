classdef EKF < handle
    properties
        X
        Dt
        x_pred

        R1
        Q1
        R2
        Q2
        R3
        Q3
        R4
        Q4
        R5
        Q5
        R6
        Q6
        P1
        P2
        P3
        P4
        P5
        P6
        K1
        K2
        K3
        K4
        K5
        K6
        debug
        
    end
    
    methods
        function obj = EKF(X0, Dt)
            obj.Dt = Dt;
            obj.x_pred = zeros(11,1);
            % filter covariances
            % AUV horizontal
            obj.R1 = 1000*eye(2); 
            obj.Q1 = kron([1,0;0,0.001],eye(2));
            % AUV Rotational
            obj.R2 = 100;
            obj.Q2 = 1;
            % DS horizontal
            obj.R3 = eye(2);
            obj.Q3 = 0.01*eye(2);
            % DS Orientation
            obj.R4 = 10;
            obj.Q4 = 0.01;
            % AUV in DS horizontal
            obj.R5 = 1000*[1,0.5;0.5,1];
            obj.Q5 = eye(2);
            % AUV in DS Orientation
            obj.R6 = 100;
            obj.Q6 = 0.01;


            % Initialization 
            obj.X = X0;
            % internal filter covariances initialization
            obj.P1 = eye(4);
            obj.P2 = 1;
            obj.P3 = eye(2);
            obj.P4 = 100;
            obj.P5 = eye(2);
            obj.P6 = 1;


            
            % solve for linear kf for horizontal filter
            obj.K1 = dlqe(eye(4)+kron([0,Dt;0,0],eye(2)), eye(4), kron([1,0],eye(2)), obj.Q1, obj.R1);
            % solve for linear kf for rotational
            obj.K2 = dlqe(1,1,1, obj.Q2, obj.R2);
            % solve for linear kf for dock yaw
            obj.K4 = dlqe(1,1,1, obj.Q4, obj.R4);
            % solve for linear kf for relative position
            obj.K5 = dlqe(eye(2),eye(2),eye(2), obj.Q5, obj.R5);
            % solve for linear kf for relative yaw
            obj.K6 = dlqe(1,1,1, obj.Q6, obj.R6);

            obj.debug = zeros(2,1);
        end
        
        function obj = predict(obj, input)
            % rotational filter
            obj.x_pred(5) = wrapTo180(obj.X(5) + input(1)*obj.Dt);
            obj.P2 = obj.P2 + obj.Q2;

            %horizontal filter
            dvl = [input(2)*cos(obj.X(5));input(2)*sin(obj.X(5))];
            A = eye(4)+kron([0,obj.Dt;0,0],eye(2));
            B = kron([obj.Dt;0],eye(2));
            obj.x_pred(1:4) = A*obj.X(1:4)+B*dvl;
            obj.P1 = A*obj.P1*A' + obj.Q1;

            % dock position estimate filter
            obj.x_pred(6:7) = obj.X(6:7);
            obj.P3 = obj.P3 + obj.Q3;
            
            % dock orientation estimate filter
            obj.x_pred(8) = obj.X(8);
            obj.P4 = obj.P4 + obj.Q4;

            % relative position estimate filter
            dvl = [input(2)*cos(obj.X(11));input(2)*sin(obj.X(11))];
            A = eye(2);
            B = obj.Dt*eye(2);
            obj.x_pred(9:10) = A*obj.X(9:10)+B*dvl;
            obj.P5 = obj.P5 + obj.Q5;

            % relative orientation estimate filter
            obj.x_pred(11) = wrapToPi(obj.X(11) + input(1)*obj.Dt);
            obj.P6 = obj.P6 + obj.Q6; 
        end

        function obj = update(obj, y)
            % horizontal filter
            C= kron([1,0],eye(2));
            obj.X(1:4) = obj.x_pred(1:4) + obj.K1*(y(1:2) - C*obj.x_pred(1:4));
            obj.P1 = (eye(4) - obj.K1*C)*obj.P1;
            
            % rotational filter
            obj.X(5) = wrapTo180(obj.x_pred(5) + obj.K2*wrapTo180((y(3) - obj.x_pred(5))));
            obj.P2 = (1 - obj.K2)*obj.P2;

            % dock position estimate filter
            d  = (obj.X(1)-obj.X(7))^2 + (obj.X(2)-obj.X(8))^2 ;
            H = [(obj.X(6)-obj.X(1))/sqrt(d), (obj.X(7)-obj.X(2))/sqrt(d);
                (obj.X(2)-obj.X(7))/d, (obj.X(6)-obj.X(1))/d];

            obj.K3 = (obj.P3*H')/(H*obj.P3*H' + obj.R3);
            y_ = y(4:5);
            hx_ = [sqrt( (obj.X(1)-obj.X(6))^2 + (obj.X(2)-obj.X(7))^2 ); wrapToPi(atan2((obj.X(7)-obj.X(2)), (obj.X(6)-obj.X(1))) - obj.X(5))];
            if ~isnan(y(5))
                obj.X(6:7) = obj.x_pred(6:7) + obj.K3*(y_-hx_);
            else
                obj.X(6:7) = obj.x_pred(6:7) + obj.K3(:,1)*(y_(1)-hx_(1));
            end
            obj.P3 = (eye(2) - obj.K3*H)*obj.P3;
            
            % dock orientation estimate filter
            usbl_b = rb2xy(y(4:5));
            usbl_ds = rb2xy(y(6:7));
            r1 = -dot(usbl_ds,usbl_b );
            r2 = [0,0,1]*cross([usbl_ds;0],[usbl_b ;0]);
            y_ = wrapToPi(-atan2(r2,r1) + obj.X(5));

            obj.X(8) = wrapToPi( obj.x_pred(8) + obj.K4*(wrapToPi(y_-obj.x_pred(8))));
            obj.P4 = (1 - obj.K4)*obj.P4;
            
            % relative position estimate filter
            y_ = (-Rot(obj.X(11))*usbl_b  + usbl_ds)/2; 
            hx_ = obj.x_pred(9:10);
            obj.X(9:10) = obj.x_pred(9:10) + obj.K5*(y_-hx_);
            obj.P5 = (eye(2) - obj.K5*H)*obj.P5;

            % relative orientation estimate filter
            r1 = -dot(usbl_ds,usbl_b);
            r2 = [0,0,1]*cross([usbl_ds;0],[usbl_b;0]); 
            y_ = atan2(r2,r1);
            obj.X(11) = wrapToPi( obj.x_pred(11) + obj.K6*(wrapToPi(y_-obj.x_pred(11))));
            obj.P6 = (1 - obj.K6)*obj.P6;
            
        end
    end
end
