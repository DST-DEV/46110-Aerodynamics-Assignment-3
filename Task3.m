clear; clc; close all;

% Constants
g = 3.728;  % Gravity on Mars [m/s^2]
rho = 14e-3;  % Atmosphere density on Mars [kg/m^3]
gamma = 1.15;  % Power correction factor

% Load results from Task 2
load('results/T2_res_sel.mat');  % Contains 'res_sel' with selected design parameters

% Battery parameters
C_bat_unit = 10/6;  % Energy capacity per battery [Wh]
m_bat_unit = 0.047;  % Mass per battery [kg]
C_bat_original = 20;  % Original battery capacity [Wh]
n_bat_original = C_bat_original / C_bat_unit;  % Number of original batteries

% Variables from Task 2
m_res = 1;  % Computer and residual components [kg]
m_pl = 2;  % Original payload mass [kg]
N_prop = res_sel.N_propellers;  % Number of propellers
L_bld = res_sel.L_blades;  % Blade length [m]
N_bld = res_sel.N_blades;  % Number of blades
A_rotor = pi * L_bld^2;  % Area of one propeller [m^2]
P_0 = res_sel.P_drag;  % Profile power [W]

% Original masses
m_bat_original = res_sel.m_batteries;  % Original battery mass [kg]
m_prop = res_sel.m_propellers;  % Propeller mass [kg]
m_mot_original = res_sel.m_motors;  % Original motor mass [kg]
m_fuse_ing = 0.3;  % Weight of the fuselage of Ingenuity [kg]
m_tot_wo_fuse_ing = 1.8 - m_fuse_ing;  % Weight of Ingenuity without the fuselage [kg]
m_mot_ing = 0.25/2;  % Weight of each propulsion motor in Ingenuity
P_prop_ing = 151.56;  % Total required power for hover [W]

% Define range of extra batteries to analyze
max_payload_mass = 2;  % Maximum mass of additional batteries [kg]
max_n_bat_extra = floor(max_payload_mass / m_bat_unit);  % Maximum number of extra batteries
n_bat_extra_range = 0:max_n_bat_extra;  % Range of extra batteries to analyze

% Initialize arrays to store results
flight_times = zeros(size(n_bat_extra_range));
total_masses = zeros(size(n_bat_extra_range));
power_required = zeros(size(n_bat_extra_range));

% Calculate flight time for each number of extra batteries
for i = 1:length(n_bat_extra_range)
    n_bat_extra = n_bat_extra_range(i);
    m_bat_extra = n_bat_extra * m_bat_unit;
    m_bat_total = m_bat_original + m_bat_extra;
    total_energy = (n_bat_original + n_bat_extra) * C_bat_unit;
    
    % Iterative calculation to account for changing motor mass
    converged = false;
    m_mot_tot = m_mot_original;  % Initial guess using original motor mass
    
    while ~converged
        m_mot_tot_prev = m_mot_tot;
        
        % Calculate total mass (replacing payload with batteries)
        m_base = m_prop + m_res + m_bat_total;  % Base mass without motors and fuselage
        m_fuse = m_fuse_ing * (m_base + m_mot_tot) / m_tot_wo_fuse_ing;  % Scale fuselage mass
        m_tot = m_base + m_mot_tot + m_fuse;
        
        % Calculate power requirements
        T_total = m_tot * g;  % Total thrust [N]
        T_rotor = T_total / N_prop;  % Thrust per rotor [N]
        
        P_ideal = (T_rotor)^(3/2) / sqrt(2 * rho * A_rotor);  % Ideal power per rotor [W]
        P_rotor = gamma * P_ideal + P_0 ;  % Total power per rotor [W]
        P_tot = N_prop * P_rotor;  % Total power [W]
        
        % Update motor mass based on power requirement
        m_mot = m_mot_ing * P_rotor / (P_prop_ing/2);  % Mass per motor [kg]
        m_mot_tot = N_prop * m_mot;  % Total motor mass [kg]
        
        % Check convergence
        if abs(m_mot_tot - m_mot_tot_prev) < 1e-4
            converged = true;
        end
    end
    
    % Store results
    total_masses(i) = m_tot;
    power_required(i) = P_tot;
    
    % Calculate flight time in minutes
    flight_times(i) = total_energy / P_tot * 60;
end

% Find the optimum number of batteries
[max_flight_time, idx_opt] = max(flight_times);
opt_n_bat_extra = n_bat_extra_range(idx_opt);
opt_mass_bat_extra = opt_n_bat_extra * m_bat_unit;

% Create figure for flight time vs number of batteries
figure;
plot(n_bat_extra_range, flight_times, '-o', 'LineWidth', 2);
hold on;
plot(opt_n_bat_extra, max_flight_time, 'p', 'MarkerSize', 12, 'LineWidth', 2, ...
     'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'r');
xlabel('Number of Extra Batteries');
ylabel('Flight Time [min]');
title('Task 3: Flight Time vs. Extra Batteries');
grid on;
text(opt_n_bat_extra + 1, max_flight_time, ...
     sprintf('Optimum: %d batteries\n%.2f minutes', opt_n_bat_extra, max_flight_time), ...
     'FontSize', 10, 'HorizontalAlignment', 'left');

% Display results
fprintf('Optimum number of extra batteries: %d\n', opt_n_bat_extra);
fprintf('Optimum flight time: %.2f minutes\n', max_flight_time);

% Check if optimum exceeds payload limit
if opt_mass_bat_extra > max_payload_mass
    fprintf('Note: The optimum exceeds the 2kg payload limit.\n');
    fprintf('Maximum allowed extra batteries: %d\n', max_n_bat_extra);
    fprintf('Flight time with max allowed batteries: %.2f minutes\n', flight_times(end));
end
