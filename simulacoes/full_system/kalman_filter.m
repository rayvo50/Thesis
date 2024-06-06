classdef kalman_filter < handle
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
        function self = kalman_filter(y0, Dt)
            self.Dt = Dt;
            self.x_pred = zeros(11,1);
 
            % horizontal filter
            self.R1 = 5000*eye(2); 
            self.Q1 = kron([1,0;0,0.01],eye(2));
            % self.R5 = 5000*[1,0.5;0.5,1];
            % self.Q5 = 0.1+eye(2);
            
            % orientation filter
            self.R2 = 10;
            self.Q2 = 0.1;

            % Initialization 
            usbl_b = rb2xy(y0(4:5));usbl_ds = rb2xy(y0(6:7));
            r1 = -dot(usbl_ds,usbl_b);
            r2 = [0,0,1]*cross([usbl_ds;0],[usbl_b;0]); 
            self.X = [usbl_ds;0;0;atan2(r2,r1)];

            % internal filter covariances initialization
            self.P1 = eye(4);
            self.P2 = 1;
            
            % solve for linear kf for horizontal filter
            self.K1 = dlqe(eye(4)+kron([0,Dt;0,0],eye(2)), eye(4), kron([1,0],eye(2)), self.Q1, self.R1);
            % solve for linear kf for rotational
            self.K2 = dlqe(1,1,1, self.Q2, self.R2);

            self.debug = zeros(1,1);
        end
        
        function self = update(self, y)
            usbl_b = rb2xy(y(4:5));
            usbl_ds = rb2xy(y(6:7));
            % usbl_b = y(4:5);
            % usbl_ds = y(6:7);

            % ORIENTATION =================================================
            % predict
            pred = wrapToPi(self.X(5) + y(8)*self.Dt);
            self.P2 = self.P2 + self.Q2; 
            
            % update
            r1 = -dot(usbl_ds,usbl_b);
            r2 = [0,0,1]*cross([usbl_ds;0],[usbl_b;0]); 
            y_ = atan2(r2,r1);
            self.debug = y_;
            if sqrt(self.X(1)^2+ self.X(2)^2) > 0
                self.X(5) = wrapToPi( pred + self.K2*(wrapToPi(y_-pred)));
                self.P2 = (1 - self.K2)*self.P2;
            else
                self.X(5) = pred;
            end


            % HORIZONTAL ==================================================
            % predict
            dvl = Rot(self.X(5))*y(9:10); 
            A = eye(4)+kron([0,self.Dt;0,0],eye(2));
            B = kron([self.Dt;0],eye(2));
            pred = self.X(1:4)+B*dvl;
            self.P1 = A*self.P1*A' + self.Q1;
            
            if sqrt(self.X(1)^2+ self.X(2)^2) > 0
                % update
                C= kron([1,0],eye(2));
                y_ = usbl_ds;
                hx_ = pred(1:2);
                self.X(1:4) = pred + self.K1*(y_-hx_);
                self.P1 = (eye(4) - self.K1*C)*self.P1;
            else
                self.X(1:4) = pred;
            end
        end
    end
end
