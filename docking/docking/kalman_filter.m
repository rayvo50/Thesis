classdef kalman_filter < handle
    properties
        X
        Dt
        x_pred

        R_xy
        Q_xy
        K_xy
        P_xy 
        R_z
        Q_z
        K_z
        P_z
        R_yaw
        Q_yaw
        K_yaw
        P_yaw

        Delta_psi
        debug
        last_ahrs_yaw
        
    end
    
    methods
        function self = kalman_filter(usbl_xy, usbl_z, usbl_psi, ahrs_yaw, Dt)
            self.Dt = Dt;
            self.last_ahrs_yaw = ahrs_yaw;
            self.Delta_psi = wrapToPi(usbl_psi-ahrs_yaw);
 
            % horizontal filter
            self.R_xy = eye(2); 
            self.Q_xy = eye(2)*0.1;
            
            % vertical filter
            self.R_z = 0.1;
            self.Q_z = 0.1;

            % orientation filter
            self.R_yaw = deg2rad(20);
            self.Q_yaw = 0.1;

            % Initialization 
            self.X = [usbl_xy;usbl_z;usbl_psi];

            % internal filter covariances initialization
            self.P_xy = eye(2);
            self.P_z = 1;
            self.P_yaw = 1;
            
            % solve for linear kf for horizontal filter644
            self.K_xy = dlqe(eye(2)*Dt, eye(2), eye(2), self.Q_xy, self.R_xy);
            % solve for linear kf for rotational
            self.K_yaw = dlqe(1,1,1, self.Q_yaw, self.R_yaw);
            % solve for linear kf for vertical
            self.K_z = dlqe(1,1,1, self.Q_z, self.R_z);

            self.debug = zeros(1,1);
        end


        function self = predict(self, dvl_uv, dvl_w, ahrs_r)
            
            % ORIENTATION =================================================
            % predict
            self.X(4) = wrapToPi(self.X(4) + ahrs_r*self.Dt);
            self.P_yaw = self.P_yaw + self.Q_yaw; 
    
            % HORIZONTAL ==================================================
            % predict
            dvl_xy = Rot(self.X(4))*dvl_uv;
            self.X(1:2) = self.X(1:2) + eye(2)*self.Dt*dvl_xy;
            self.P_xy = self.P_xy + self.Q_xy;

            % VERTICAL =================================================
            % predict
            self.X(3) = self.X(3) + dvl_w*self.Dt;
            self.P_z = self.P_z + self.Q_z; 
        end
        

        function self = update_usbl(self, usbl_xy, usbl_z, usbl_psi)
            
            
            % ORIENTATION =================================================  
            self.Delta_psi = wrapToPi(usbl_psi-self.last_ahrs_yaw);   %TODO: this delta_psi could have a dynamic of its own 

            self.X(4) = wrapToPi( self.X(4) + self.K_yaw*(wrapToPi(usbl_psi-self.X(4))));
            self.P_yaw = (1 - self.K_yaw)*self.P_yaw;

            % HORIZONTAL ==================================================
            self.X(1:2) = self.X(1:2) + self.K_xy*(usbl_xy-self.X(1:2));
            self.P_xy = (eye(2) - self.K_xy)*self.P_xy;

            % VERTICAL ====================================================
            self.X(3) = wrapToPi( self.X(3) + self.K_z*(wrapToPi(usbl_z-self.X(3))));
            self.P_z = (1 - self.K_z)*self.P_z;
        end    


        function self = update_ahrs(self, ahrs_psi)

            % ORIENTATION =================================================
            self.last_ahrs_yaw = ahrs_psi;
            dock_psi = wrapToPi(ahrs_psi + self.Delta_psi);

            self.X(4) = wrapToPi(self.X(4) + self.K_yaw*(wrapToPi(dock_psi-self.X(4))));
            self.P_yaw = (1 - self.K_yaw)*self.P_yaw;          
        end


        

    end
end
