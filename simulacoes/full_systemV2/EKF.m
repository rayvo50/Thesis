classdef EKF < handle
    properties
        X
        Dt

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
        function self = EKF(X0, Dt)
            self.Dt = Dt;
            x_pred = zeros(11,1);
            % filter covariances
            % AUV horizontal
            self.R1 = 1000*eye(2); 
            self.Q1 = kron([1,0;0,0.001],eye(2));
            % AUV Rotational
            self.R2 = 100;
            self.Q2 = 1;
            % DS horizontal
            self.R3 = 1*eye(2);
            self.Q3 = 0.01*eye(2);
            % DS Orientation
            self.R4 = 10;
            self.Q4 = 0.01;
            % AUV in DS horizontal
            self.R5 = 1000*[1,0.5;0.5,1];
            self.Q5 = eye(2);
            % AUV in DS Orientation
            self.R6 = 100;
            self.Q6 = 0.01;


            % Initialization 
            self.X = X0;
            % internal filter covariances initialization
            self.P1 = eye(4);
            self.P2 = 1;
            self.P3 = 100*eye(2);
            self.P4 = 100;
            self.P5 = eye(2);
            self.P6 = 1;


            
            % solve for linear kf for horizontal filter
            self.K1 = dlqe(eye(4)+kron([0,Dt;0,0],eye(2)), eye(4), kron([1,0],eye(2)), self.Q1, self.R1);
            % solve for linear kf for rotational
            self.K2 = dlqe(1,1,1, self.Q2, self.R2);
            % solve for linear kf for dock yaw
            self.K4 = dlqe(1,1,1, self.Q4, self.R4);
            % solve for linear kf for relative position
            self.K5 = dlqe(eye(2),eye(2),eye(2), self.Q5, self.R5);
            % solve for linear kf for relative yaw
            self.K6 = dlqe(1,1,1, self.Q6, self.R6);

            self.debug = zeros(4,1);
        end
        
        function self = compute(self, input,y)
            x_pred = zeros(11,1);
            % AUV rotational ==============================================
            x_pred(5) = wrapTo180(self.X(5) + input(1)*self.Dt);
            self.P2 = self.P2 + self.Q2;

            self.X(5) = wrapTo180(x_pred(5) + self.K2*wrapTo180((y(3) - x_pred(5))));
            self.P2 = (1 - self.K2)*self.P2;


            % AUV horizontal ==============================================
            dvl = [input(2)*cosd(self.X(5));input(2)*sind(self.X(5))];
            A = eye(4)+kron([0,self.Dt;0,0],eye(2));
            B = kron([self.Dt;0],eye(2));
            x_pred(1:4) = A*self.X(1:4)+B*dvl;
            self.P1 = A*self.P1*A' + self.Q1;

            C= kron([1,0],eye(2));
            self.X(1:4) = x_pred(1:4) + self.K1*(y(1:2) - C*x_pred(1:4));
            self.P1 = (eye(4) - self.K1*C)*self.P1;



            % DS position =================================================
            x_pred(6:7) = self.X(6:7);
            self.P3 = self.P3 + self.Q3;

            d  = (self.X(1)-self.X(6))^2 + (self.X(2)-self.X(7))^2 ;
            H = [(self.X(6)-self.X(1))/sqrt(d), (self.X(7)-self.X(2))/sqrt(d);
                (self.X(2)-self.X(7))/d, (self.X(6)-self.X(1))/d];

            self.K3 = (self.P3*H')/(H*self.P3*H' + self.R3);
            y_ = y(4:5);
            hx_ = [sqrt( (self.X(1)-self.X(6))^2 + (self.X(2)-self.X(7))^2 ); wrapTo180(atan2d((self.X(7)-self.X(2)), (self.X(6)-self.X(1))) - self.X(5))];
            if ~isnan(y(5))
                self.X(6:7) = x_pred(6:7) + self.K3*(y_-hx_);
            else
                self.debug = [y_(1);hx_(1);self.K3(:,1)];
                self.X(6:7) = x_pred(6:7) + self.K3(:,1)*(y_(1)-hx_(1));
            end
            self.P3 = (eye(2) - self.K3*H)*self.P3;
            
            % DS orientation ==============================================
            if ~isnan(y(5))
                x_pred(8) = self.X(8);
                self.P4 = self.P4 + self.Q4;
    
                usbl_b = rb2xy(y(4:5));
                usbl_ds = rb2xy(y(6:7));
                r1 = -dot(usbl_ds,usbl_b );
                r2 = [0,0,1]*cross([usbl_ds;0],[usbl_b ;0]);
                y_ = wrapTo180(-atan2d(r2,r1) + self.X(5));
                self.X(8) = wrapTo180( x_pred(8) + self.K4*(wrapTo180(y_-x_pred(8))));
                self.P4 = (1 - self.K4)*self.P4;

            % AUV relative position =======================================
                dvl = [input(2)*cosd(self.X(11));input(2)*sind(self.X(11))];
                A = eye(2);
                B = self.Dt*eye(2);
                x_pred(9:10) = A*self.X(9:10)+B*dvl;
                self.P5 = self.P5 + self.Q5;

                y_ = (-Rot(self.X(11))*usbl_b  + usbl_ds)/2; 
                hx_ = x_pred(9:10);
                self.X(9:10) = x_pred(9:10) + self.K5*(y_-hx_);
                self.P5 = (eye(2) - self.K5*H)*self.P5;
    
            % AUV relative orientation ====================================
                x_pred(11) = wrapTo180(self.X(11) + input(1)*self.Dt);
                self.P6 = self.P6 + self.Q6; 

                r1 = -dot(usbl_ds,usbl_b);
                r2 = [0,0,1]*cross([usbl_ds;0],[usbl_b;0]); 
                y_ = atan2d(r2,r1);
                self.X(11) = wrapTo180( x_pred(11) + self.K6*(wrapTo180(y_-x_pred(11))));
                self.P6 = (1 - self.K6)*self.P6;
            end
          
            
        end
    end
end