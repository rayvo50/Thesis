classdef homing_controller < handle
    properties
        state
        Delta
        surge_controller
        yaw_controller
        y_controller
        z_controller
        output
        s;
        Phis;
        e;
        first_it
        lpf_state;
        lpf_Ad
        lpf_Bd 
        Dt;
        ref;
        yaw_ref_filter;
        z_ref_filter;
        y_ref_filter;
    end
    
    methods
        function self = homing_controller()

            self.state = 0;                         % Initial PF aproach 
            self.surge_controller = surge_PID();
            self.yaw_controller = yaw_SMC();
            self.y_controller = y_SMC();
            self.z_controller = z_SMC();
            self.Delta = 0.5;
            self.output=zeros(4,1);
            self.s=zeros(4,1);
            self.Dt = 0.1;
            
            % References LPFs
            wc = 1;

            % %first order lpf
            % a = -wc;
            % b = wc;
            % self.lpf_ad = exp(a*self.Dt);
            % self.lpf_bd = (1/a)*(lpf_ad-1)*b;

            % 2nd order lpf
            A = [0,1;-wc^2,-sqrt(2)*wc];
            B = [0;wc^2];
            self.lpf_Ad = expm(A*self.Dt);
            self.lpf_Bd = A\(self.lpf_Ad-eye(2))*B;
            self.first_it =true;
            self.lpf_state = [0;0];

            self.ref = nan(6,1);


        end
        
        function self = compute(self, x_hat, dvl, ahrs)
            x = x_hat(1); y = x_hat(2); z = x_hat(3); yaw = x_hat(4);
            u = dvl(1); v = dvl(2); w = dvl(3); r = ahrs; 
            
            if self.first_it
                self.yaw_ref_filter = TrackingReferenceFilter(self.Dt, 1.0, yaw, true);
                self.z_ref_filter = TrackingReferenceFilter(self.Dt, 1.0, z, false);
                self.y_ref_filter = TrackingReferenceFilter(self.Dt, 1.0, y, false);
                self.first_it=false;
            end

            % Hysteresis-based mode switching
            if abs(y) > 5
                self.state = 0;
                self.first_it=true;
            elseif abs(y) < 2
                self.state = 1;
                self.first_it=true;
            end
            
            
            % LOS guidance
            if self.state == 0 
                yaw_d = atan2(-y,-self.Delta);
                u_d = 0.2;
                z_d = 0;
                y_d = nan;

            % terminal line following
            elseif self.state == 1
                yaw_d = -pi;
                u_d = 0.1;
                z_d = 0;
                y_d = 0;
            end
            
            
            tau = zeros(4,1);
            % u reference for PID
            self.surge_controller.compute(u, u_d);
            tau(1) = self.surge_controller.output;
            
            % y reference for SMC
            vcy=0.0;
            %= compute(self, y, y_ref, y_rate_ref, y_rate_rate_ref, yaw, u, v, vcy)
            tau(2)=0;
            y_ref=nan;
            if ~isnan(y_d)
                [y_ref, y_rate_ref, y_acc_ref] = self.y_ref_filter.update(y_d);
                self.y_controller.compute(y, y_d, 0, 0, yaw, u, v, r, vcy);
                tau_y = self.y_controller.output;
                tau_y_body = Rot(-yaw)*[0; -tau_y];
                %tau(1) = tau(1)+tau_y_body(1);
                tau(2) = -tau_y_body(2);
            else
                self.y_ref_filter.y(1) = y;
            end
            % z reference for SMC
            [z_ref, z_rate_ref, z_acc_ref] = self.z_ref_filter.update(z_d);
            self.z_controller.compute(z, w, z_d, 0,0);
            tau(3) =self. z_controller.output;
            
            % Yaw reference for SMC
            [yaw_ref, yaw_rate_ref, yaw_acc_ref] = self.yaw_ref_filter.update(yaw_d);
            self.yaw_controller.compute(yaw, r, yaw_d, 0, 0, u, v);
            tau(4) =self. yaw_controller.output;
            
            self.output = tau;
            self.s = [0;self.y_controller.s;self.z_controller.s;self.yaw_controller.s];
            self.Phis = [0;self.y_controller.Phi;self.z_controller.Phi;self.yaw_controller.Phi];
            self.e = [self.surge_controller.e;self.y_controller.e;self.z_controller.e;self.yaw_controller.e];
            self.ref = [u_d; y_d; z_d; yaw_d; z_rate_ref; z_acc_ref];
        end
        

      
    end
end
