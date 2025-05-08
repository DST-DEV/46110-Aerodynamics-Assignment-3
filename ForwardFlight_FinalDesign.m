%% TASK 6: FORWARD FLIGHT. OPTIMAL WINGSPAN: NO BLADE
clear 
clc
close all

% DATA LOADING
load AG14_14k.txt % XFOIL results
aoa = AG14_14k(:,1);
cl = AG14_14k(:,2);
cd = AG14_14k(:,3);

% VARIABLE INPUTS
AR = 10;
vinf = linspace(0,12,25); % [m/s] Ground speed

% INPUTS
g = 3.73; % [m/s2] Gravitational force
density = 0.015; % [kg/m3]
Nrotors = 4; % Number of rotors
R = 0.7; % [m] Rotor blade length
Ar = pi*R^2; % [m2] Rotor area

% DRAG CALCULATIONS
Cdb = 0.4; % Drag coefficient of drone body
Ad = 0.3; % Area of drone body % ASSUMPTION

for ii = 1:length(vinf)

q = 0.5*density*vinf(ii)^2; % Dynamic pressure

%% MASS CALCULATIONS
m = 5.03; % [kg] Drone mass with payload, from T2

%% DRAG CALCULATIONS
D = q*Ad*Cdb; % Drag of drone body

%% TOTAL THRUST
beta = atan(D/(m*g)); % [rad] Tilt angle
tilt(ii) = beta;

T = m*g/cos(beta); % [N] Total thrust
Tr = T/Nrotors; % Thrust per rotor

%% INDUCED VELOCITY
vH = sqrt(Tr/(2*density*Ar)); % [m/s] Hover velocity
% Solving the equation
F = @(vi) (vH^2/sqrt((vinf(ii)*cos(beta))^2 + (vinf(ii)*sin(beta) + vi)^2)) - vi;
vi0 = 0.1; % Initial guess
vi = fsolve(F, vi0);

%% POWER CALCULATIONS
kappa = 1.15; % Induced power correction factor
Nb = 2; % Number of blades
R_ing = 0.6; % [m] Original ingenuity rotor blade length
w = sqrt(Tr/7.43e-5); % [rad/s] Based on the values of previous tasks

Pid = T*(vinf(ii)*sin(beta) + vi); % [W] Rotor ideal power
k = 0.000001044044;   % P0 coefficient from BEM
P0 = (k*w^3)*Nrotors;

P(ii) = kappa*Pid + P0; % [W] Total power
end


%% PLOTTING THE RESULTS
gray = [0.7,0.7,0.7];
figure(1)
ax = gca;
colororder([0 0 0; 0.4 0.4 0.4])
yyaxis left
plot(vinf,P,LineWidth=1.5,LineStyle="-")
set(ax,'FontSize',16);
ylabel('Power [W]', 'Interpreter', 'latex')
yyaxis right
plot(vinf,rad2deg(tilt),LineWidth=1.5, LineStyle=":")
set(ax,'FontSize',16);
ylabel('Tilt angle [deg]', 'Interpreter', 'latex')

xlabel('Flight speed [m/s]', 'Interpreter', 'latex')
grid on
















%% FUNCTIONS

% Function to calculate the Fourier components
function An = Fourier_components(b, m0, c, theta, beta, beta_L0, N)
    % Left hand side
    for jj = 1:length(theta)
        for ii = 1:N
            AA(jj,ii) = (4*b)/(m0*c(jj))*sin(ii*theta(jj)) + ...
                        ii*sin(ii*theta(jj))/sin(theta(jj));
        end
    end
    
    % Right hand side
    BB = beta*ones(length(theta),1) - beta_L0*ones(length(theta),1);
    % Calculating the Fourier components
    An = AA\BB;
end

% ------------------------------------------------

% Function to calculate the induced angle of attack
function aoa_i = induced_aoa(theta, An, N)

 % Calculating the induced angle of attack
for jj = 1:length(theta)
    for ii = 1:N
        alpha_i(ii) = ii.*An(ii)*sin(ii*theta(jj))/sin(theta(jj));
    end

    % Calculating the induced angle of attack
    aoa_i(jj) = sum(alpha_i);
end

end

