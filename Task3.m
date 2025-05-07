clear; clc; close all;

% Constants
g = 3.728;               % Mars gravity [m/s^2]
rho = 14e-3;             % Mars air density [kg/m^3]
gamma = 1.15;            % Correction factor
C_bat_unit = 10/6;       % Battery capacity [Wh]
m_bat_unit = 0.047;      % Battery mass [kg]
max_payload_mass = 2;    % Payload mass limit [kg]

% Load selected drone results
load('results/T2_res_sel.mat');
n_bat_original = 20 / C_bat_unit;  % Assume 20 Wh default battery pack

% Drone mass breakdown
m_bat_orig = res_sel.m_batteries;
m_mot = res_sel.m_motors;
m_prop = res_sel.m_propellers;
m_fuse = res_sel.m_fuselage;
m_res = 1;  % Reserve structure
N_prop = res_sel.N_propellers;
L_bld = res_sel.L_blades;
A_rotor = pi * L_bld^2;
P_0 = res_sel.P_drag;

%% SIMULATION 1: 0–100 BATTERIES 
n1 = 0:100;
t1 = zeros(size(n1));
m_bat_total1 = zeros(size(n1));

for i = 1:length(n1)
    n_extra = n1(i);
    m_extra = n_extra * m_bat_unit;
    m_bat_total = m_bat_orig + m_extra;
    m_total = m_mot + m_prop + m_fuse + m_res + m_bat_total;
    T = m_total * g;
    T_rotor = T / N_prop;
    P_rotor = gamma * T_rotor^(3/2) / sqrt(2 * rho * A_rotor) + P_0;
    P_total = N_prop * P_rotor;
    E_total = (n_bat_original + n_extra) * C_bat_unit;
    t1(i) = E_total / P_total * 60;  % min
    m_bat_total1(i) = m_extra;
end

% Find max within 2kg
valid_mask = m_bat_total1 <= max_payload_mass;
[max_valid_time, idx_valid] = max(t1(valid_mask));
n_valid_opt = n1(valid_mask);
n_valid_opt = n_valid_opt(idx_valid);
m_valid_opt = m_bat_total1(n1 == n_valid_opt);

%% SIMULATION 2: 0–200 BATTERIES
n2 = 0:200;
t2 = zeros(size(n2));

for i = 1:length(n2)
    n_extra = n2(i);
    m_extra = n_extra * m_bat_unit;
    m_bat_total = m_bat_orig + m_extra;
    m_total = m_mot + m_prop + m_fuse + m_res + m_bat_total;
    T = m_total * g;
    T_rotor = T / N_prop;
    P_rotor = gamma * T_rotor^(3/2) / sqrt(2 * rho * A_rotor) + P_0;
    P_total = N_prop * P_rotor;
    E_total = (n_bat_original + n_extra) * C_bat_unit;
    t2(i) = E_total / P_total * 60;
end

[max_theo_time, idx_theo] = max(t2);
n_theo_opt = n2(idx_theo);
m_theo_opt = n_theo_opt * m_bat_unit;

%% PLOT 1: Up to 100 batteries 
figure;
plot(n1, t1, 'o-', 'LineWidth', 1.8); hold on;
plot(n_valid_opt, max_valid_time, 'g*', 'MarkerSize', 10, 'LineWidth', 2);
xlabel('Number of Extra Batteries');
ylabel('Flight Time [min]');
title('Flight Time vs. Extra Batteries (0–100)');
legend('Flight Time', 'Max Within 2kg Limit', 'Location', 'southeast');
grid on;
xlim([0 100]);
ylim([min(t1) max(t1)+2]);

text(n_valid_opt+2, max_valid_time, ...
    sprintf('Valid Max: %d batteries\n%.2f min (%.2f kg)', ...
    n_valid_opt, max_valid_time, m_valid_opt), ...
    'FontSize', 9);

%% PLOT 2: Up to 200 batteries
figure;
plot(n2, t2, 'o-', 'LineWidth', 1.8); hold on;
plot(n_theo_opt, max_theo_time, 'r*', 'MarkerSize', 10, 'LineWidth', 2);
xlabel('Number of Extra Batteries');
ylabel('Flight Time [min]');
title('Flight Time vs. Extra Batteries (0–200)');
legend('Flight Time', 'Theoretical Maximum', 'Location', 'southeast');
grid on;
xlim([0 200]);
ylim([min(t2) max(t2)+2]);

text(n_theo_opt+2, max_theo_time, ...
    sprintf('Theoretical Max: %d batteries\n%.2f min (%.2f kg)', ...
    n_theo_opt, max_theo_time, m_theo_opt), ...
    'FontSize', 9);

%% Print results
fprintf(' Optimum within 2 kg limit: \n');
fprintf('Valid Max: %d extra batteries\nFlight time: %.2f min\nBattery mass: %.2f kg\n\n', ...
    n_valid_opt, max_valid_time, m_valid_opt);

fprintf('Theoretical optimum: \n');
fprintf('Theoretical Max: %d extra batteries\nFlight time: %.2f min\nBattery mass: %.2f kg\n', ...
    n_theo_opt, max_theo_time, m_theo_opt);
