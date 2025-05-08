%% TASK 6: FORWARD FLIGHT. OPTIMAL WINGSPAN
clear 
clc
close all

% DATA LOADING
load AG14_14k.txt % XFOIL results
aoa = AG14_14k(:,1);
cl = AG14_14k(:,2);
cd = AG14_14k(:,3);

% VARIABLE INPUTS
b_length = 4;
b = linspace(0.01,b_length,b_length*10);
AR = [5, 10, 15];

% INPUTS
g = 3.73; % [m/s2] Gravitational force
density = 0.015; % [kg/m3]
Nrotors = 4; % Number of rotors
R = 0.7; % [m] Rotor blade length
Ar = pi*R^2; % [m2] Rotor area
vinf = 10; % [m/s] Ground speed
alpha = 6; % [deg] Angle of attack of the wing
alpha = deg2rad(alpha);

% DRAG CALCULATIONS
Cdb = 0.4; % Drag coefficient of drone body
e = 0.8; % Oswald efficiency factor [0.7-0.9]
Ad = 0.3; % Area of drone body % ASSUMPTION

% LIFTING LINE THEORY
N = 200; % Number of divisions
n = (1:N)';
theta = linspace(pi/N, pi, N); % Transformed x coordinates
m0 = 5.67; % [m] Local lift slope AG14, taken from XFOIL
alpha_L0 = deg2rad(-1.778); % Angle of attack for Lift = 0, from XFOIL

q = 0.5*density*vinf^2; % Dynamic pressure

for jj = 1:length(AR)
for ii = 1:length(b)
    % ASSUMPTIONS
    c = b(ii)/AR(jj);
    S = b(ii)*c; % [m2] Wing area
    
    %% MASS CALCULATIONS
    m_d = 5.03; % [kg] Drone mass with payload, from T2
    density_w = 74; % [kg/m3] Wing density
    
    t = 0.088*c; % Thickness of the airfoil
    % vol_w = S*c;
    vol_w =b(ii)*c*t; % [m3] Wing volume
    m_w = vol_w*density_w; % [kg] Wing mass
    
    m = m_d + m_w; % Total mass
    weight(ii,jj) = m;
    
    %% LIFTING LINE THEORY: RECTANGULAR BLADE
    % Calculating the Fourier components
    An = Fourier_components(b(ii), m0, c*ones(1,N), theta, alpha, alpha_L0, N);
    % Calculating the lift and induced drag coefficients
    Clw = pi*AR(jj)*An(1,:); % Lift coefficient of wing
    Cdi = pi*AR(jj)*sum(n.*(An.^2), 1); % Induced drag coefficient of wing
    
    Lw = q*S*Clw; % [N] Wing lift force
    
    %% DRAG CALCULATIONS
    Db = q*Ad*Cdb; % Drag of drone body
    
    % Wing drag
    Cd = interp1(aoa, cd, rad2deg(alpha), "linear"); % Friction drag coef
    Dw = q*S*Cdi +  q*c*Cd*b(ii); % [N] Wing drag
    
    D = Db + Dw; % [N] Total drag
    
    %% TOTAL THRUST
    beta = atan(D/(m*g)); % [rad] Tilt angle
    L = m*g - Lw; % [N] Required lift
    if L < 0
        L = 0;
    end
    T = L/cos(beta); % [N] Total thrust
    Tr = T/Nrotors; % Thrust per rotor
    
    %% INDUCED VELOCITY
    vH = sqrt(Tr/(2*density*Ar)); % [m/s] Hover velocity
    % Solving the equation
    F = @(vi) (vH^2/sqrt((vinf*cos(beta))^2 + (vinf*sin(beta) + vi)^2)) - vi;
    vi0 = 0.1; % Initial guess
    vi = fsolve(F, vi0);
    
    %% POWER CALCULATIONS
    kappa = 1.15; % Induced power correction factor
    Nb = 2; % Number of blades
    R_ing = 0.6; % [m] Original ingenuity rotor blade length
    w = sqrt(Tr/7.43e-5); % [rad/s] Based on the values of previous tasks
    omega(ii,jj) = w;
    
    Pid = T*(vinf*sin(beta) + vi); % [W] Rotor ideal power
    k = 0.000001044044;   % P_0 coefficient from BEM
    P0 = (k*w^3)*Nrotors;
    
    P(ii,jj) = kappa*Pid + P0; % [W] Total power
end
end


%% PLOTTING THE RESULTS
figure(1)
ax = gca;
plot(b,P(:,1), '-k',LineWidth=1.5)
hold on
plot(b,P(:,2), '--k',LineWidth=1.5)
plot(b,P(:,3), ':k',LineWidth=1.5)
hold off
set(ax,'FontSize',16);
xlabel('Wingspan [m]', 'Interpreter', 'latex')
ylabel('Power [W]', 'Interpreter', 'latex')
grid on
legend('AR = 5','AR = 10','AR = 15', 'Interpreter', 'latex', 'Location','northwest')
set(ax, 'TickLabelInterpreter', 'latex');

figure(2)
ax = gca;
plot(b,weight(:,1), '-k', LineWidth=1.5)
hold on
plot(b,weight(:,2), '--k',LineWidth=1.5)
plot(b,weight(:,3), ':k',LineWidth=1.5)
hold off
set(ax,'FontSize',16);
xlabel('Wingspan [m]', 'Interpreter', 'latex')
ylabel('Weight [kg]', 'Interpreter', 'latex')
grid on
legend('AR = 5','AR = 10','AR = 15', 'Interpreter', 'latex', 'Location','northwest')
set(ax, 'TickLabelInterpreter', 'latex');










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

