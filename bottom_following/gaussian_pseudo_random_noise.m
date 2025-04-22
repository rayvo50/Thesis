classdef gaussian_pseudo_random_noise < handle
    % Generates a pseudo-random noise profile along a distance x
    % Works by generating a vector with gaussian random noise and then
    % passing it through a low pass filter. Then this sequence is stored in
    % the class.
    % For every x that the function is called, the previous sequence is
    % interpolated to give the value of the profile at coordinate x.
    
    properties
        x_reference         % initial grid of coordinates 
        correlated_noise    % noise profile
        y                   % output (profile value at given coordinate)
    end
    
    methods
        % Constructor
        function self = gaussian_pseudo_random_noise()
            seed = 42;          % Fixed seed for reproducibility
            x_min = -1;         % Define the domain for the function
            x_max = 50;
            num_points = 2000;  % Number of points in the reference grid
            sigma = 0.3;        % Correlation length scale (smaller = noisier)
            
            % Fix the random seed for repeatability
            rng(seed);
            
            % Generate a reference grid of equally spaced points
            self.x_reference = linspace(x_min, x_max, num_points);
            
            % Generate correlated noise using a Gaussian kernel
            % Compute pairwise distances
            dist_matrix = abs(self.x_reference' - self.x_reference);
            % Compute kernel matrix (Gaussian kernel)
            kernel_matrix = exp(-dist_matrix.^2 / (2 * sigma^2));
            % Generate random noise with the specified correlation
            base_noise = randn(num_points, 1); % Independent Gaussian noise
            self.correlated_noise = kernel_matrix * base_noise;
            
            % Normalize the correlated noise
            self.correlated_noise = self.correlated_noise - mean(self.correlated_noise);
            self.correlated_noise = self.correlated_noise / std(self.correlated_noise);
            
            % Add variability with multi-scale noise
            for i = 1:9
                % Add finer-scale noise by reducing sigma and blending
                finer_sigma = sigma / (2^i);
                finer_kernel_matrix = exp(-dist_matrix.^2 / (2 * finer_sigma^2));
                finer_noise = finer_kernel_matrix * randn(num_points, 1);
                finer_noise = finer_noise - mean(finer_noise); % Normalize
                self.correlated_noise = self.correlated_noise + finer_noise / (2^i); % Blend
            end
        end
        
        % Function to compute the value of the noise profile
        function y = compute(self, x)
            y = interp1(self.x_reference, self.correlated_noise, x, 'linear', 'extrap');
        end
    end
end
