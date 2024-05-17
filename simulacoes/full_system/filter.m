classdef filter < handle
    properties
        X
        Dt
        x_pred

        R1
        Q1
        R2
        Q2
        P1
        P2
        K1
        K2
        
        debug
        
    end
    
    methods
        function obj = filter(X0, Dt)
            obj.Dt = Dt;
            obj.x_pred = zeros(11,1);
 
            % horizontal filter
            obj.R1 = 5000*eye(2); 
            obj.Q1 = kron([1,0;0,0.001],eye(2));
            % obj.R5 = 5000*[1,0.5;0.5,1];
            % obj.Q5 = 0.1+eye(2);
            
            % orientation filter
            obj.R2 = 1;
            obj.Q2 = 0.1;

            % Initialization 
            obj.X = X0;
            % internal filter covariances initialization
            obj.P1 = eye(4);
            obj.P2 = 1;
            
            % solve for linear kf for horizontal filter
            obj.K1 = dlqe(eye(4)+kron([0,Dt;0,0],eye(2)), eye(4), kron([1,0],eye(2)), obj.Q1, obj.R1);
            % solve for linear kf for rotational
            obj.K2 = dlqe(1,1,1, obj.Q2, obj.R2);

            obj.debug = zeros(2,1);
        end
        
        function obj = update(obj, y)
            usbl_b = rb2xy(y(4:5));
            usbl_ds = rb2xy(y(6:7));

            % ORIENTATION ===========================
            % predict
            pred = wrapToPi(obj.X(5) + y(1)*obj.Dt);
            obj.P2 = obj.P2 + obj.Q2; 

            % update
            r1 = -dot(usbl_ds,usbl_b);
            r2 = [0,0,1]*cross([usbl_ds;0],[usbl_b;0]); 
            y_ = atan2(r2,r1);
            obj.X(5) = wrapToPi( pred + obj.K2*(wrapToPi(y_-pred)));
            obj.P2 = (1 - obj.K2)*obj.P2;

            % HORIZONTAL ======================================
            % predict
            dvl = Rot(pred)*y(2:3); 
            A = eye(4)+kron([0,obj.Dt;0,0],eye(2));
            B = kron([obj.Dt;0],eye(2));
            pred = A*obj.X(1:4)+B*dvl;
            obj.P1 = A*obj.P1*A' + obj.Q1;
            
            % update
            C= kron([1,0],eye(2));
            y_ = usbl_ds;
            hx_ = obj.x_pred(1:2);
            obj.X(1:4) = pred + obj.K1*(y_-hx_);
            obj.P1 = (eye(4) - obj.K1*C)*obj.P1;
     
        end
    end
end
