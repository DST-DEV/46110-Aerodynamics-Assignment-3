clc; clear;

% Constants
g = 3.73;                       % Mars gravity [m/s^2]
rho = 20e-3;                    % Mars atmospheric density [kg/m^3]
C_d0 = 0.02;                    % Profile drag coefficient
omega = 2800 * 2 * pi / 60;     % Angular velocity [rad/s]
c = 0.1;                        % Mean blade chord [m]
Nb = 2;                         % Number of blades per rotor
R = 0.6;                        % Rotor radius [m]
m = 1.8;                        % Total mass [kg]
gamma = 1.15;                   % Correction factor

% Area
A = pi * R^2;                   % Rotor area [m^2]

% Define corrected system of equations
equations = @(x) [
    2*rho*A*x(1)^2 + 2*rho*A*(x(1)+x(2))*x(2) - m*g;                      % Thrust balance
    (2*rho*A*x(1)^2)*x(1) - (2*rho*A*(x(1)+x(2))*x(2))*(x(1)+x(2))         % Power equality
];

% Initial guess: [v1, v2]
x0 = [1; 0.5];

% Solve using fsolve
opts = optimoptions('fsolve','Display','off');
solution = fsolve(equations, x0, opts);

% Extract values
v1 = solution(1);
v2 = solution(2);

% Calculate thrusts and powers
T1 = 2 * rho * A * v1^2;
T2 = 2 * rho * A * (v1 + v2) * v2;
P1_ideal = T1 * v1;
P2_ideal = T2 * (v1 + v2);



% Total Power Calculations  
%P1_ideal = T1^(1.5) / sqrt(2 * rho * A);                      % Ideal hover power [W] for rotor1
%P2_ideal = T2^(1.5) / sqrt(2 * rho * A);                      % Ideal hover power [W] for rotor2
P_0 = (1/8) * rho * c * Nb * C_d0 * omega^3 * R^4;             % Profile drag power [W]
P1_total = gamma * P1_ideal + P_0; 
P2_total = gamma * P2_ideal + P_0;
P_total = P1_total + P2_total;

% Display results
fprintf('v1 = %.3f m/s\n', v1);
fprintf('v2 = %.3f m/s\n', v2);
fprintf('T1 = %.3f N\n', T1);
fprintf('T2 = %.3f N\n', T2);
fprintf('Power per rotor 1: %.2f W\n', P1_total);
fprintf('Power per rotor 2: %.2f W\n', P2_total);
fprintf('Total hover power (both rotors): %.2f W\n', P_total);
