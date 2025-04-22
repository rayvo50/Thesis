% Define the ODE: x' = 2
dxdt = @(t, x) -1*sign(x);

% Initial condition
x0 = 2;

% Time span
tspan = [0 10];

% Solve using ode45
[t, x] = ode45(dxdt, tspan, x0);

% Plot the solution
figure;
plot(t, x, 'b-', 'LineWidth', 2);
xlabel('Time t');
ylabel('x(t)');
title('Solution of dx/dt = 2 with x(0) = 0');
grid on;
