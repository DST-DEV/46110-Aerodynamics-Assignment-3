clear; clc; 
close all;
%% Preparations
%User inputs
drag = true;  % Selection whether drag should be considered
savefigs = true;
res_fld = 'results';
plot_fld = 'plots';

% Constants
g = 3.728;  % Gravity on Mars [m/s^2]
rho = 14e-3;  % Atmosphere density on Mars [kg/m^3]

% Original Ingenuity design
R_ing = .6;  % Propeller radius of Ingenuity [m]
omega_ing = 2800 * 2*pi / 60;  % Rotational speed [rad/s]
r_ing = [0.0908, 0.1158, 0.1551, 0.2000, 0.2472, 0.2950, 0.3429, 0.3903, ...
    0.4369, 0.4826, 0.5271, 0.5704, 0.6121, 0.6523, 0.6908, 0.7274, 0.7621, ...
    0.7947, 0.8252, 0.8535, 0.8794, 0.9030, 0.9241, 0.9427, 0.9588, 0.9722, ...
    0.9830, 0.9912, 0.9966, 1.0000] .* R_ing;
dr_ing = r_ing - [0, r_ing(1:end-1)];
c_ing = [0.0506, 0.0631, 0.0967, 0.1407, 0.1758, 0.1968, 0.2021, 0.1968, ...
    0.1863, 0.1743, 0.1627, 0.1530, 0.1446, 0.1375, 0.1314, 0.1259, 0.1209, ...
    0.1160, 0.1111, 0.1058, 0.1001, 0.0935, 0.0860 ,0.0775, 0.0679, 0.0573, ...
    0.0460, 0.0341, 0.0223, 0.0123] .* R_ing;

m_tot_ing = 1.8;  % Total weight of Ingenuity [kg]
m_mot_ing = .25/2;  % Weight of each propulsion motor in Ingenuity
m_fuse_ing = .3;  % Weight of the fuselage of Ingenuity [kg]
m_tot_wo_fuse_ing = m_tot_ing - m_fuse_ing;  % Weight of Ingenuity without the fuselage [kg]
P_prop_ing = 151.56;  % Total required power for hover [W] 

% Parameter variations
N_prop = [2, 4];  % Number of propellers
N_bld = 2:4;  % Number of blades of each propeller
L_bld = .6:.05:1.25;  % Length of blades [m]
A_prop = N_prop' .* pi .* L_bld.^2;  % Total area of all propellers
omega = omega_ing .* R_ing ./ L_bld;  % Rotational speed

% Assumptions
C_d0 = .02;  % Drag coefficient
c = mean(c_ing, Weights=dr_ing)*L_bld./R_ing;  % Mean chord length [m]
gamma = 1.15;  % Power correction factor

% Known parameters of the new design
C_bat = 20;  % Battery capacity [Wh]

% Masses of new design
m_prop = 70e-3/4 .* N_prop' .* L_bld/R_ing .* reshape(N_bld, 1, 1, []);
m_res = 1;  % Weight of computer and residual components [kg]
m_bat = .5; % Weight of the battery pack [kg]
m_pl = 2;  % Weight of the payload

m_base = m_prop + m_res + m_bat + m_pl;  % Base weight of the new design (WITH payload)

% Drag power
P_0 = 1/8 * rho .* c .* N_prop' .* reshape(N_bld, 1, 1, []) .* C_d0 .* omega.^3 .* L_bld.^4;

%% Determine total mass

% Initial guess for the weight of the fuselage [kg]
m_fuse = m_fuse_ing .* m_base/m_tot_wo_fuse_ing;  

% Initial guess for the weight of the propulsion motors [kg]
P_ideal = ((m_base + m_fuse).*g) .^ 1.5 ./ (2 * rho * A_prop);
if drag
    P = gamma * P_ideal + P_0;
else
    P = gamma * P_ideal;
end
m_mot = m_mot_ing * P / P_prop_ing;

m_tot_0 = m_tot_ing;
m_tot = m_base + m_fuse + m_mot;

i = 0;
while ~all(abs(m_tot-m_tot_0)<.05,'all')  % Hella inefficient but does the job
    i = i+1;
    %Update total mass from previous iteration
    m_tot_0 = m_tot;

    % Calculate the required power
    P_ideal = (m_tot.*g) .^ 1.5 ./ (2 * rho * A_prop);
    if drag
        P = gamma * P_ideal + P_0;
    else
        P = gamma * P_ideal;
    end

    % Calculate mass of motors
    m_mot = m_mot_ing .* P / P_prop_ing;
    
    % Calculate mass of fuselage
    m_fuse = m_fuse_ing .* (m_base + m_mot) ./ m_tot_wo_fuse_ing;  

    % Calculate new total mass
    m_tot = m_base + m_fuse + m_mot;
end
%% Post calculations

% Calculate the required power and mass of rotors one last time
P_base_ideal = (m_tot.*g) .^ 1.5 ./ (2 * rho * A_prop);
if drag
    P_base = gamma * P_base_ideal + P_0;
else
    P_base = gamma * P_base_ideal;
end
m_mot = m_mot_ing .* P / P_prop_ing;

% Calculate final total mass (with payload) and power consumption with
% payload
m_tot = m_base + m_fuse + m_mot;
P_wo_pl_ideal = ((m_tot - m_pl).*g) .^ 1.5 ./ (2 * rho * A_prop);
P_w_pl_ideal = (m_tot.*g) .^ 1.5 ./ (2 * rho * A_prop);
if drag
    P_w_pl = gamma * P_w_pl_ideal + P_0;
else
    P_w_pl = gamma * P_w_pl_ideal;
end

% Calculate flight time [min]
t_flight = C_bat./P_w_pl .* 60;

%% Plot findings
fig_index = 1;
if savefigs
    [fig_m, ax_m, fig_index] = plot_blade_designs (m_tot-m_pl, N_bld, L_bld, ...
        fig_index, "T2_m_wo_pl", "$m$ [kg]", plot_fld, savefigs);
    [fig_base, ax_P_base, fig_index] = plot_blade_designs (P_wo_pl_ideal, N_bld, ...
        L_bld, fig_index, "T2_P_base", "$P$ [W]", plot_fld, savefigs);
    [fig_P_w_pl, ax_P_w_pl, fig_index] = plot_blade_designs (P_w_pl, N_bld, L_bld, ...
        fig_index, "T2_P_w_pl", "$P$ [W]", plot_fld, savefigs);
    [fig_t, ax_t, fig_index] = plot_blade_designs (t_flight, N_bld, L_bld, ...
        fig_index, "T2_t_flight_w_pl", "$t$ [min]", plot_fld, savefigs);
else
    [fig_m, ax_m, fig_index] = plot_blade_designs (m_tot-m_pl, N_bld, L_bld, ...
        fig_index, "Total mass excl. payload", "$m$ [kg]", plot_fld, savefigs);
    [fig_base, ax_P_base, fig_index] = plot_blade_designs (P_wo_pl_ideal, N_bld, ...
        L_bld, fig_index, "Power for hovering (without payload and drag)", ...
        "$P$ [W]", plot_fld, savefigs);
    [fig_P_w_pl, ax_P_w_pl, fig_index] = plot_blade_designs (P_w_pl, N_bld, L_bld, ...
        fig_index, "Power for hovering (with payload)", ...
        "$P$ [W]", plot_fld, savefigs);
    [fig_t, ax_t, fig_index] = plot_blade_designs (t_flight, N_bld, L_bld, ...
        fig_index, "Flight time with payload", "$t$ [min]", plot_fld, savefigs);
end

function [fig, ax, fig_index] = plot_blade_designs(data, numBlades, ...
        bladeLengths, fig_index, figTitle, ylabel_txt, plot_fld, savefig)
    % Validate dimensions
    if ndims(data) ~= 3 || size(data, 1) ~= 2
        error('Data must be a 2 x m x n array.');
    end

    if length(numBlades) ~= size(data, 3) || length(bladeLengths) ~= size(data, 2)
        error('Vector dimensions must match the 2nd and 3rd dimensions of data.');
    end
    
    if nargin < 8, savefig = false; end

    cols = ["#0072BD", "#D95319", "#EDB120", "#77AC30", "#80B3FF"];  % Colors of the lines
    markers = ["none", "none", "none", "none", "none"];  % Markers for the four methods
    ms = [4.5, 4.5, 4.5, 4.5, 4.5];  % Marker size for the plots of the four methods
    lw = [1.5, 1.5, 1.5, 1.5, 1.5];  % Linewidth for the lines of the four methods
    ax_col = [0.2, 0.2, 0.2];  % Color of accented axes
    ax_lw = 1.5;  % Line width of accented axes
    fs = 16;  % Plot font size

    % Extract data for each design
    doublePropellerData = squeeze(data(1, :, :));
    quadcopterData = squeeze(data(2, :, :));

    % Plotting

    cmap = "abyss";
    fig = figure(fig_index); grid on;
    fig_index = fig_index + 1;
    set(gcf, 'Position', [100, 100, 800, 400]);
    cla; hold on; grid on;
    ax = gca;
    
    for i = 1:numel(numBlades)
        lines_tandem(i) = plot(bladeLengths, ...
                               doublePropellerData(:,i), ...
                               LineWidth=lw(i), Marker=markers(i), ...
                               MarkerSize=ms(i), LineStyle='-',...
                               Color=cols(i), ...
                               DisplayName=sprintf('$N_{bld}=%d$', ...
                                                   numBlades(i)));
        lines_quad(i) = plot(bladeLengths, ...
                             quadcopterData(:,i), ...
                             LineWidth=lw(i), Marker=markers(i), ...
                             MarkerSize=ms(i), LineStyle='--', ...
                             Color=cols(i), ...
                             DisplayName=sprintf('$N_{bld}=%d$', ...
                                                 numBlades(i)));
    end

    % Plot labels
    set(gcf,'Color','White');
    set(ax,'FontSize',fs);
    ylabel(ylabel_txt, 'Interpreter', 'latex');
    xlabel('$L_{bld}$ [m]', 'Interpreter', 'latex');
    set(ax, 'TickLabelInterpreter', 'latex');
    xlim(ax, [min(bladeLengths), max(bladeLengths) + .03]);

    lgd1 = legend(lines_tandem, 'Interpreter', 'latex', ...
                  'Location', 'northeastoutside');
    lgd1.Title.String = 'Tandem Design';
    
    % Force graphics update to get position
    drawnow;
    
    % Get legend 1 position [left bottom width height]
    pos1 = lgd1.Position;
    
    % Calculate position for second legend just below
    spacing = 0.01; % small gap between legends
    new_bottom = pos1(2) - pos1(4) - spacing;

    % Create a second, invisible axes for the second legend
    ax2 = axes('Position', get(gca, 'Position'), 'Color', 'none');
    h = plot(nan, nan, '--r', nan, nan, '--b'); % dummy lines
    lgd2 = legend(ax2, lines_quad, 'Interpreter', 'latex');
    lgd2.Title.String = 'Quadcopter Design';
    lgd2.Position = [pos1(1), new_bottom, pos1(3), pos1(4)];
    axis off; % hide the dummy axes
    set(gca, 'FontSize', fs);        % Axis ticks and labels

    if savefig
        fpath = fullfile(plot_fld, figTitle + '.pdf');
        exportgraphics(gcf, fpath, 'ContentType', 'vector', ...
            'BackgroundColor', 'none', 'Resolution', 300);
    else
        title(gca, figTitle, 'Interpreter', 'latex');
    end
end

%% Select design

% Selection of values
N_prop_sel = 4;
N_bld_sel = 2;
L_bld_sel = 1;

i_sel = [find(N_prop == N_prop_sel), find(L_bld == L_bld_sel), ...
    find(N_bld == N_bld_sel)]; %Index of the selected values

% Retrieve required power for hovering for selected parameters
P_base_sel = P_base(i_sel(1), i_sel(2), i_sel(3));
P_w_pl_sel = P_w_pl(i_sel(1), i_sel(2), i_sel(3));

% Retrieve masses for selected parameters
m_tot_sel = m_tot(i_sel(1), i_sel(2), i_sel(3));
m_sel = [m_prop(i_sel(1), i_sel(2), i_sel(3)), ...
         m_bat, ...
         m_mot(i_sel(1), i_sel(2), i_sel(3)), ...
         m_fuse(i_sel(1), i_sel(2), i_sel(3)), ...
         m_res];
m_sel_dist = m_sel / m_tot_sel * 100;
t_flight_sel = t_flight(i_sel(1), i_sel(2), i_sel(3));

% Prepare labels for pie chart
mass_labels = {sprintf('Rotors: %.1f g', m_sel(1)*1e3), ...
               sprintf('Battery pack: %.1f g', m_sel(2)*1e3), ...
               sprintf('Propulsion and control motors: %.1f g', m_sel(3)*1e3), ...
               sprintf('Fuselage: %.1f g', m_sel(4)*1e3), ...
               sprintf('Computer and other components: %.1f g', m_sel(5)*1e3), ...
               };

% Export results
res = struct('N_prop', N_prop_sel, 'N_bld', N_bld_sel, ...
    'L_bld_sel', L_bld_sel, ...
    'P_base', P_base_sel, 'P_w_pl', P_w_pl_sel, ...
    'm_tot', m_tot_sel, 'm_prop', m_sel(1), 'm_bat', m_sel(2), ...
    'm_mot', m_sel(3), 'm_fuse', m_sel(4), 'm_res', m_sel(4), ...
    't_flight', t_flight_sel);
save(fullfile(res_fld, 'T2_res.mat'), 'res');

figure(fig_index); 
set(gcf, 'Position', [100, 100, 1000, 450]);
pc = piechart(m_sel, mass_labels, 'FontSize', 15, 'FontName', 'times');
pc.LabelStyle="name";
% pc.ColorOrder = abyss(numel(m_sel));
pc.ColorOrder = slanCM('blues', numel(m_sel));
if savefigs
    fpath = fullfile(plot_fld, 'T2_mass_dist.pdf');
    exportgraphics(gcf, fpath, 'ContentType', 'vector', ...
        'BackgroundColor', 'none', 'Resolution', 300);
end
