classdef TrackingReferenceFilter < handle
    properties
        y                  % [value; derivative]
        y_dd               % second derivative (acceleration)
        wn                 % natural frequency
        zeta               % damping ratio
        Dt                 % timestep
        wrap_angle         % boolean: whether to wrap the signal to [-pi, pi]
    end

    methods
        function self = TrackingReferenceFilter(Dt, wn, y0,wrap_angle)
            self.wn = wn;                % natural frequency (tune this)
            self.zeta = 1.0;              % damping ratio (1 = critical damping)
            self.Dt = Dt;                 % sampling period
            self.wrap_angle = wrap_angle; % whether to wrap input/output
            self.y = [y0; 0];              % [signal_ref; signal_dot]
            self.y_dd = 0;                % signal_ddot
        end

        function [ref, ref_dot, ref_ddot] = update(self, signal_d)
            % error with optional angle wrap
            if self.wrap_angle
                err = wrapToPi(self.y(1) - signal_d);
            else
                err = self.y(1) - signal_d;
            end

            % second-order low-pass filter (spring-damper system)
            dy1 = self.y(2);
            dy2 = -2*self.zeta*self.wn*self.y(2) - self.wn^2 * err;

            % integrate
            self.y(1) = self.y(1) + self.Dt * dy1;
            if self.wrap_angle
                self.y(1) = wrapToPi(self.y(1));
            end

            self.y(2) = self.y(2) + self.Dt * dy2;
            self.y_dd = dy2;

            % outputs
            ref = self.y(1);
            ref_dot = self.y(2);
            ref_ddot = self.y_dd;
        end
    end
end

% 
% classdef TrackingReferenceFilter < handle
%     properties
%         zeta;
%         wn;
%         Dt;
%         state; % [r; r_dot]
%         wrap = false;
%     end
% 
%     methods
%         function self = TrackingReferenceFilter(Dt, wn, y0,wrap_angle)
%             self.wn = wn;
%             self.zeta = 0.7;
%             self.Dt = Dt;
%             self.wrap = wrap_angle;
%             self.state = [y0; 0]; % initialize with zero
%         end
% 
%         function [r, r_dot, r_ddot] = update(self, r_raw)
%             if self.wrap
%                 e = wrapToPi(r_raw - self.state(1));
%             else
%                 e = r_raw - self.state(1);
%             end
% 
%             % second order tracking dynamics
%             r_ddot = -2*self.zeta*self.wn*self.state(2) - self.wn^2*e;
%             self.state(2) = self.state(2) + self.Dt*r_ddot;
%             self.state(1) = self.state(1) + self.Dt*self.state(2);
% 
%             r = self.state(1);
%             r_dot = self.state(2);
%         end
%     end
% end
