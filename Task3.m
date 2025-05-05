clear; clc; close all;

% Load results from Task 2
load('results/T2_res_full.mat');  % Contains 'res'
load('results/T2_res_sel.mat');   % Contains 'res_sel'

% Re-declare these parameter arrays for index lookups (same as in Task 2)
N_prop = [2, 4];                % Number of propellers
N_bld = 2:4;                    % Number of blades
L_bld = 0.4:0.05:1.25;          % Blade lengths
g = 3.728;                     % Mars gravity
rho = 14e-3;                   % Mars atmosphere density
gamma = 1.15;                  % Power correction factor
C_bat_unit = 10 / 6;           % Wh per battery
m_bat_unit = 0.047;            % kg per battery
m_pl = 2;                      % Original payload (replaced with batteries)
m_fuse_ing = 0.3;              % From Task 2
m_tot_wo_fuse_ing = 1.8 - m_fuse_ing;

% Index of selected design (quadcopter, L=0.7, blades=2)
i_quad = find(N_prop == 4);
i_bld = find(N_bld == 2);
i_len = find(abs(L_bld - 0.7) < 1e-3);

% Extract necessary base data
m_base = res.m_base(i_quad, i_len, i_bld);
P_0_sel = res.P_0(i_quad, i_len, i_bld);
A_rotor = pi * L_bld(i_len)^2;

% Battery sweep
max_payload_mass = 2;  % Max extra battery mass
max_n_bat = floor(max_payload_mass / m_bat_unit);
battery_counts = 0:max_n_bat;
flight_times = zeros(1, length(battery_counts));

for n = battery_counts
    m_bat_ext = n * m_bat_unit;
    E_total = n * C_bat_unit;

    m_new_base = m_base + m_bat_ext - m_pl;  % Replace payload with batteries
    m_new_fuse = m_fuse_ing * m_new_base / m_tot_wo_fuse_ing;
    m_tot = m_new_base + m_new_fuse;

    T_total = m_tot * g;
    T_rotor = T_total / 4;

    P_ideal = (T_rotor)^(3/2) / sqrt(2 * rho * A_rotor);
    P_rotor = gamma * P_ideal + P_0_sel;
    P_total = 4 * P_rotor;

    % Flight time [min]
    flight_times(n + 1) = (E_total / P_total) * 60;
end

% Plot
figure;
plot(battery_counts, flight_times, '-o', 'LineWidth', 2);
xlabel('Number of Extra Batteries');
ylabel('Flight Time [min]');
title('Task 3: Flight Time vs Extra Batteries');
grid on;

% Find optimum
[max_time, idx_opt] = max(flight_times);
opt_bat = battery_counts(idx_opt);

fprintf(' Optimum number of batteries: %d\n', opt_bat);
fprintf(' Flight time at optimum: %.2f min\n', max_time);

% If too heavy
if opt_bat * m_bat_unit > max_payload_mass
    max_fit = floor(max_payload_mass / m_bat_unit);
    time_max_fit = flight_times(max_fit + 1);
    fprintf(' Optimum exceeds 2kg battery limit.\n');
    fprintf(' Max allowed batteries: %d\n', max_fit);
    fprintf(' Flight time with allowed max: %.2f min\n', time_max_fit);
end
