classdef outlier_rejection
    % Function to do outlier rejection on a signal vector y of dimension n
    % This is implemented as a class in order to store the buffer
    % It works by aplying a MAD criterion to the data on a rolling window. 
    % For reference see Morgado et all 2014

    properties
        K           % Window size
        buffer      % Circular buffer for the signal
        idx         % Current index in the buffer
        count       % Number of elements in the buffer
        threshold   % decision threshold for z score
        y_filtered  % output signal without outliers 
        debug
    end

    methods
        % Constructor
        function self = outlier_rejection(K, threshold, n) 
            self.K = K;
            self.buffer = nan(n, K); % Initialize buffer for all channels
            self.idx = 1;
            self.count = 0;
            self.threshold = threshold;
            self.debug=zeros(2,1);
        end

        function [self, y_filtered]  = compute(self, y)
            y_filtered = y;
            n=size(y,1);

            % Update buffer
            self.buffer(:, self.idx) = y;
            self.idx = mod(self.idx, self.K) + 1;
            self.count = min(self.count + 1, self.K);
            % Check if we have enough samples
            if self.count < self.K
                return;
            end

            % Compute median and MAD
            medians = median(self.buffer, 2); % Median for each channel
            
            mad_values = median(abs(self.buffer - medians), 2) / 0.6745; % MAD per channel

            % Identify outliers
            robust_z_scores = abs(y - medians) ./ mad_values;
            self.debug= medians;
            is_outliers = robust_z_scores > self.threshold;
            if is_outliers(1) ==true || is_outliers(2) ==true
                if 0
                    disp("Outlier rejected")
                end
            end    
            
            % Replace outliers with median
            y_filtered(is_outliers) = medians(is_outliers);
        end
    end
end
