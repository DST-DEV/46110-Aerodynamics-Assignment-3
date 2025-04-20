clear; clc; 
close all;
%% Preparations
%User inputs
drag = true;  % Selection whether drag should be considered

% Constants
g = 3.728;  % Gravity on Mars [m/s^2]
rho = 20e-3;  % Atmosphere density on Mars [kg/m^3]

% Assumptions
C_d0 = .02;  % Drag coefficient
omega = 2800 * 2*pi / 60;  % Rotational speed [rad/s]
c = .12;  % Mean chord length [m]
gamma = 1.15;  % Power correction factor

% Original Ingenuity design
R_ing = .6;  % Propeller radius of Ingenuity [m]
m_tot_ing = 1.8;  % Total weight of Ingenuity [kg]
m_mot_ing = .25/2;  % Weight of each propulsion motor in Ingenuity
m_fuse_ing = .3;  % Weight of the fuselage of Ingenuity [kg]
m_tot_wo_fuse_ing = m_tot_ing - m_fuse_ing;  % Weight of Ingenuity without the fuselage [kg]
P_prop_ing = 170;  % Total required power for hover [W] 
%-------------- NOTE: P_prop_ing is a guess!!!!!!!!!!!!!!!!!!!!!!!!!!!!

% Known parameters of the new design
C_bat = 20;  % Battery capacity [Wh]

% Parameter variations
N_prop = [2, 4];  % Number of propellers
N_bld = 2:8;  % Number of blades of each propeller
L_bld = .6:.05:1.25;  % Length of blades [m]
A_prop = N_prop' .* pi .* L_bld.^2;  % Total area of all propellers

% Masses of new design
m_prop = 70e-3/4 .* N_prop' .* L_bld/R_ing .* reshape(N_bld, 1, 1, []);
m_res = 1;  % Weight of computer and residual components [kg]
m_bat = .5; % Weight of the battery pack [kg]
m_pl = 2;  % Weight of the payload

m_base = m_prop + m_res + m_bat + m_pl;  % Base weight of the new design (WITH payload)

% Drag power
P_0 = 1/8 * rho * c * N_prop' .* reshape(N_bld, 1, 1, []) .* C_d0 .* omega.^3 .* L_bld.^4;

%% Determine total mass
% Note: This calculation does not consider the payload (just the weight of
% the drone itself)

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

ax_m = plot_blade_designs (m_tot-m_pl, N_bld, L_bld, 1,...
                           "Total mass excl. payload", "m [kg]");
ax_P_base = plot_blade_designs (P_wo_pl_ideal, N_bld, L_bld, 2,...
                                "Power for hovering (without payload and drag)", ...
                                 "P [W]");
ax_P_w_pl = plot_blade_designs (P_w_pl, N_bld, L_bld, 3,...
                                "Power for hovering (with payload)", ...
                                "P [W]");
ax_t = plot_blade_designs (t_flight, N_bld, L_bld, 4,...
                           "Flight time with payload", "t [min]");

function axes = plot_blade_designs(data, numBlades, bladeLengths, fig,...
    figTitle, colorbarLabel)
    % Validate dimensions
    if ndims(data) ~= 3 || size(data, 1) ~= 2
        error('Data must be a 2 x m x n array.');
    end

    if length(numBlades) ~= size(data, 3) || length(bladeLengths) ~= size(data, 2)
        error('Vector dimensions must match the 2nd and 3rd dimensions of data.');
    end

    if nargin < 4
        figTitle = '';  % Default to empty if not provided
    end

    if nargin < 5
        colorbarLabel = '';  % Default to empty if not provided
    end

    % Create meshgrid for plotting
    [X, Y] = meshgrid(bladeLengths, numBlades);

    % Extract data for each design
    doublePropellerData = squeeze(data(1, :, :));
    quadcopterData = squeeze(data(2, :, :));

    % Plotting
    figure(fig); grid on;
    set(gcf, 'Position', [100, 100, 800, 400]);
    cmap = "abyss";

    % Double Propeller Design
    ax1 = subplot(1,2,1);
    contourf(ax1, X, Y,doublePropellerData', 'LineColor', 'none');
    colormap(ax1, cmap);
    cb1 = colorbar(ax1);
    ylabel(cb1, colorbarLabel);
    xlabel(ax1, 'Blade Length [m]', 'Interpreter', 'latex');
    ylabel(ax1, 'Number of Blades', 'Interpreter', 'latex');
    title(ax1, 'Double Propeller Design', 'Interpreter', 'latex');
    axis square;
    set(ax1, 'TickLabelInterpreter', 'latex');

    % Quadcopter Design
    ax2 = subplot(1,2,2);
    contourf(ax2, X, Y, quadcopterData', 'LineColor', 'none');
    colormap(ax2, cmap);
    cb2 = colorbar(ax2);
    ylabel(cb2, colorbarLabel);
    xlabel(ax2, 'Blade Length [m]', 'Interpreter', 'latex');
    ylabel(ax2, 'Number of Blades', 'Interpreter', 'latex');
    title(ax2, 'Quadcopter Design', 'Interpreter', 'latex');
    axis square;
    set(ax2, 'TickLabelInterpreter', 'latex');

    % Optional figure title
    if ~isempty(figTitle)
        sgtitle(figTitle, 'Interpreter', 'latex');
    end

    axes = {ax1, ax2};
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

% Prepare labels for pie chart
mass_labels = {sprintf('Rotors: %.1f g', m_sel(1)*1e3), ...
               sprintf('Battery pack: %.1f g', m_sel(2)*1e3), ...
               sprintf('Propulsion and control motors: %.1f g', m_sel(3)*1e3), ...
               sprintf('Fuselage: %.1f g', m_sel(4)*1e3), ...
               sprintf('Computer and other components: %.1f g', m_sel(5)*1e3), ...
               };

figure(5); 
set(gcf, 'Position', [100, 100, 1000, 800]);
pc = piechart(m_sel, mass_labels);
pc.LabelStyle="name";
% pc.ColorOrder = abyss(numel(m_sel));
pc.ColorOrder = slanCM('blues', numel(m_sel));

%% Add design selection to pts

draw_crosshair(ax_m{i_sel(1)}, L_bld_sel, N_bld_sel)
draw_crosshair(ax_P_base{i_sel(1)}, L_bld_sel, N_bld_sel)
draw_crosshair(ax_P_w_pl{i_sel(1)}, L_bld_sel, N_bld_sel)
draw_crosshair(ax_t{i_sel(1)}, L_bld_sel, N_bld_sel)

function draw_crosshair(ax, x, y, style, width, color)
    % Set default values if not provided
    if nargin < 4 || isempty(style), style = '--'; end
    if nargin < 5 || isempty(width), width = 1; end
    if nargin < 6 || isempty(color), color = '#c5c5c5'; end

    % Hold the axis
    hold(ax, 'on');

    % Draw horizontal line
    yline(ax, y, style, 'LineWidth', width, 'Color', color);

    % Draw vertical line
    xline(ax, x, style, 'LineWidth', width, 'Color', color);
end
