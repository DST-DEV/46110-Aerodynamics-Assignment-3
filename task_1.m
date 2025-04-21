clear; clc;
close all;

%% Task1 – Power consumed by each rotor in hover 
% Constants
g = 3.728;                  % Mars gravity [m/s^2]
rho = 20e-3;                % Mars atmospheric density [kg/m^3]
C_d0 = 0.02;                % Profile drag coefficient
omega = 2800 * 2 * pi / 60; % Rotor angular velocity [rad/s]
gamma = 1.15;               % Power correction factor (from Task 2)
c = 0.03;                   % Estimated average blade chord [m]
Nb = 2;                     % Number of blades per rotor
R = 0.6;                    % Rotor radius [m]
A = pi * R^2;               % Rotor disk area [m^2]

% Total helicopter parameters
m_total = 1.8;              % Ingenuity mass [kg]
T_rotor = (m_total * g) / 2;  % Thrust per rotor [N]

% Rotor solidity (used in profile drag power)
sigma = Nb * c / (pi * R);  % Rotor solidity [-]

% Induced power (from momentum theory)
P_induced = (T_rotor^1.5) / sqrt(2 * rho * A);

% Profile power (from blade-element theory approximation)
P_profile = (1/8) * rho * C_d0 * omega^3 * R^5 * sigma;

% Total power per rotor (including correction factor)
P_per_rotor = gamma * P_induced + P_profile;

%% Output
fprintf('Estimated power consumed by each rotor: %.2f W\n', P_per_rotor);
