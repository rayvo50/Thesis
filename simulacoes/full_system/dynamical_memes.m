% Parameters initialization
mu = 1; mv = 1; mr = 1; du = 0.1; dv = 0.1; dr = 0.1;
tau_u = 0.5; tau_v = 0.2; tau_r = 0.1; w = 0; % Assuming constant values for simplicity
Dt = 0.1; % Time step
T = 10; % Total simulation time
N = T/Dt; % Number of simulation points

% State initialization
u = zeros(1, N+1); v = zeros(1, N+1); r = zeros(1, N+1);
psi = zeros(1, N+1); x = zeros(1, N+1); y = zeros(1, N+1);

x = zeros(size(0:Dt:T));

% Simulation loop
for n = 1:N
    c(n+1) = u(n) + Dt * (1/mu * (tau_u + mv * v(n) * r(n) - du * u(n)));
    v(n+1) = v(n) + Dt * (1/mv * (tau_v + mu * u(n) * r(n) - dv * v(n)));
    r(n+1) = r(n) + Dt * (1/mr * (tau_r + muv * u(n) - dr * r(n)));
    psi(n+1) = psi(n) + Dt * r(n);
    x(n+1) = x(n) + Dt * (u(n) * cos(psi(n)) - v(n) * sin(psi(n)));
    y(n+1) = y(n) + Dt * (u(n) * sin(psi(n)) + v(n) * cos(psi(n)));
end

% Plotting the trajectory
plot(x, y);
xlabel('X Position');
ylabel('Y Position');
title('ASV Path Following Simulation');
grid on;
