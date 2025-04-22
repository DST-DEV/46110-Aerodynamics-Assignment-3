clear; clc;
close all;

%% Task 1 – Power per rotor in hover (Ingenuity, Mars)

% === Constants ===
g = 3.73;                      % Mars gravity [m/s^2]
rho = 20e-3;                    % Mars atmospheric density [kg/m^3]
C_d0 = 0.02;                    % Profile drag coefficient
omega = 2800 * 2 * pi / 60;     % Angular velocity [rad/s]
c = 0.1;                        % Mean blade chord [m]
Nb = 2;                         % Number of blades per rotor
R = 0.6;                        % Rotor radius [m]
m = 1.8;                        % Total mass [kg]
gamma = 1.15;                   % Correction factor

% === Area===
A = pi * R^2;                   % Rotor disk area [m^2]

% === Thrust ===
T = (m * g)/2;                  % Thrust per rotor [N]


% === Power Calculations ===
P_ideal = T^(1.5) / sqrt(2 * rho * A);                      % Ideal hover power [W]
P_0 = (1/8) * rho * c * Nb * C_d0 * omega^3 * R^4;          % Profile drag power [W]
P_total = gamma * P_ideal + P_0;                            % Total power per rotor [W]

% === Output ===
fprintf('Ideal hover power per rotor       : %.2f W\n', P_ideal);
fprintf('Profile drag power per rotor      : %.2f W\n', P_0);
fprintf('Total hover power per rotor       : %.2f W\n', P_total);
