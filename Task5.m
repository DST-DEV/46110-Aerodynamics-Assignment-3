clear; clc; close all;
%% User inputs
savefigs = true;
res_fld = 'results';
plot_fld = 'plots';
rerun_opt = false;

% Optimization variations
c_tip = .05:.005:.09;
c_root = linspace(.06, .1, numel(c_tip));
r_thres = .3;
r_tip = .97;

alpha_D = -1:.5:4;

%% Plot settings
cols = ["#0072BD", "#D95319", "#EDB120", "#77AC30", "#80B3FF"];  % Colors of the lines
lw = [1, 1, 1, 1, 1];  % Linewidth for the lines of the four methods
ax_col = [0.2, 0.2, 0.2];  % Color of accented axes
ax_lw = 1.5;  % Line width of accented axes
fs = 16;  % Plot font size
fig_index = 1;

%% Setup
% Results from Task 2
N_prop = 4;  % Number of propellers
N_bld = 2;  % Number of blades per rotor
R = 1;  % Rotor radius [m].
m_tot = 4.93;  % Total mass of the drone (incl. payload) [kg]

r = 0:.01:R;

optimizer = ShapeOptimizer(N_prop, N_bld, R, m_tot);
if rerun_opt
    [res_opt, res] = optimizer.optimize(r, alpha_D, c_tip, c_root, ...
        r_thres, r_tip);
    
    save(fullfile(res_fld, 'T5_opt_res_full.mat'), 'res');
    save(fullfile(res_fld, 'T5_opt_res_final.mat'), 'res_opt');
else
    res = load(fullfile(res_fld, 'T5_opt_res_full.mat')).res;
    res_opt = load(fullfile(res_fld, 'T5_opt_res_final.mat')).res_opt;

    r = res.r;
    c_tip = res.c_tip;
    c_root = res.c_root;
    r_thres = res.r_thres;
    r_tip = res.r_thres;
    alpha_D = res.alpha_D;
end

%% Plot c_tip vs alpha_D as contour plot

% Create meshgrid for plotting
[X, Y] = meshgrid(c_tip, alpha_D);

% Plot power
cmap = "abyss";
figure(fig_index); grid on;
fig_index = fig_index + 1;
set(gcf, 'Position', [100, 100, 400, 400]);

contourf(gca, X, Y, res.P', 'LineColor', 'none');
colormap(gca, cmap);
cb = colorbar(gca);
ylabel(cb, 'Total power [W]', 'Interpreter', 'latex');
xlabel(gca, 'Tip chord [m]', 'Interpreter', 'latex');
ylabel(gca, 'Design AoA [$^{\circ}$]', 'Interpreter', 'latex');
axis square;
set(gca, 'TickLabelInterpreter', 'latex');
set(cb, 'TickLabelInterpreter', 'latex');

if savefigs
    exportgraphics(gcf, 'T5_power_opt_contourf.pdf', 'ContentType', 'vector', ...
        'BackgroundColor', 'none', 'Resolution', 300);
end

% Plot Thrust
cmap = "abyss";
figure(fig_index); grid on;
fig_index = fig_index + 1;
set(gcf, 'Position', [100, 100, 400, 400]);

contourf(gca, X, Y, res.T', 'LineColor', 'none');
colormap(gca, cmap);
cb = colorbar(gca);
ylabel(cb, 'Total thrust [N]', 'Interpreter', 'latex');
xlabel(gca, 'Tip chord [m]', 'Interpreter', 'latex');
ylabel(gca, 'Design AoA [$^{\circ}$]', 'Interpreter', 'latex');
axis square;
set(gca, 'TickLabelInterpreter', 'latex');
set(cb, 'TickLabelInterpreter', 'latex');

if savefigs
    exportgraphics(gcf, 'T5_thrust_opt_contourf.pdf', 'ContentType', 'vector', ...
        'BackgroundColor', 'none', 'Resolution', 300);
end

%% Plot final chord distribution
c_opt = res_opt.c_tip ./ r(r/R>=r_thres);

% Calculate chord near root 
c_root = (c_opt(1)-res_opt.c_root)/(R*res_opt.r_thres) * r(r/R<r_thres) + res_opt.c_root;

% Combine the two curves
c = horzcat(c_root, c_opt);

% Smoothen out the sharp peak between the two curves and the wing tip
c_cut = 2.5*res_opt.c_tip;
idx_valid = c<=c_cut & r./R<=res_opt.r_tip;
r_valid = r(idx_valid);
c_valid = c(idx_valid);
if r_tip<1
    r_valid(end+1) = R;
    c_valid(end+1) = 0;
end

% Plot wing shape
figure(fig_index);
fig_index = fig_index + 1;
resizeFigure(gcf, 800, 400);

cla; hold on; grid on;
plot (r/R, res_opt.c_tip ./ r, LineWidth=lw(1), LineStyle='--', Color='k', ...
    DisplayName='Ideal chord');
plot(r, c, LineWidth=lw(2)*1.5, LineStyle='-.', Color=cols(2), ...
    DisplayName='Raw chord distribution'); 
plot (r_valid, c_valid, LineWidth=lw(3)*1.5, LineStyle='-.', Color=cols(3), ...
    DisplayName='Filtered chord distribution');
plot (r/R, squeeze(res_opt.c), LineWidth=lw(1), LineStyle='-', Color='k', ...
    DisplayName='Final chord distribution');
hold off;

set(gcf,'Color','White');
set(gca,'FontSize',fs);
ylabel('$c$ [m]', 'Interpreter', 'latex');
xlabel('$r/R$', 'Interpreter', 'latex');
set(gca, 'TickLabelInterpreter', 'latex');
xlim(gca, [0, 1]);
ylim(gca, [0, max(res_opt.c) + .05]);
legend('Interpreter', 'latex', 'Location', 'northeast');

if savefigs
    fpath = fullfile(plot_fld, 'T5_c_vs_r.pdf');
    exportgraphics(gcf, fpath, 'ContentType', 'vector', ...
        'BackgroundColor', 'none', 'Resolution', 300);
end

%% Plot final twist distribution

theta_opt = res_opt.alpha_D + rad2deg(.5./r(r/R>=res_opt.r_thres) .* sqrt(optimizer.C_T_req));

% Calculate chord near root
theta_root = zeros(1, numel(r(r/R<res_opt.r_thres))) + theta_opt(1);

%Combine the two curves
theta = horzcat(theta_root, theta_opt);

% Smoothen out the sharp corner between the two curves
idx_valid = r<=(r_thres-.05) | r>=(r_thres+.05);
r_valid = r(idx_valid);
theta_valid = theta(idx_valid);

% Plot twist distribution
figure(fig_index);
fig_index = fig_index + 1;
resizeFigure(gcf, 800, 400);

cla; hold on; grid on;
plot (r/R, res_opt.alpha_D + rad2deg(.5./r .* sqrt(optimizer.C_T_req)), ...
    LineWidth=lw(1), LineStyle='--', Color='k', DisplayName='Ideal twist');
plot(r, theta, LineWidth=lw(2)*1.5, LineStyle='-.', Color=cols(2), ...
    DisplayName='Raw twist distribution'); 
plot (r_valid, theta_valid, LineWidth=lw(3)*1.5, LineStyle='-.', Color=cols(3), ...
    DisplayName='Filtered twist distribution');
plot(r/R, squeeze(res_opt.theta), LineWidth=lw(1), LineStyle='-', Color='k', ...
    DisplayName='Final twist distribution')
hold off;

set(gcf,'Color','White');
set(gca,'FontSize',fs);
ylabel('$\theta$ [$^{\circ}$]', 'Interpreter', 'latex');
xlabel('$r/R$', 'Interpreter', 'latex');
set(gca, 'TickLabelInterpreter', 'latex');
xlim(gca, [0, 1]);
ylim(gca, [min(res_opt.theta)-1, max(res_opt.theta) + 1]);
legend('Interpreter', 'latex', 'Location', 'northeast');

if savefigs
    fpath = fullfile(plot_fld, 'T5_theta_vs_r.pdf');
    exportgraphics(gcf, fpath, 'ContentType', 'vector', ...
        'BackgroundColor', 'none', 'Resolution', 300);
end

%% Plot dC_T
dC_T = res_opt.dT ./ (.5*optimizer.rho.*optimizer.A_rotor.*(optimizer.omega.*R).^2);

figure(fig_index);
fig_index = fig_index + 1;
resizeFigure(gcf, 800, 400);
cla; hold on; grid on;

plot(r/R, squeeze(dC_T), LineWidth=lw(1), LineStyle='-', Color='k')

set(gcf,'Color','White');
set(gca,'FontSize',fs);
ylabel('$dC_T$', 'Interpreter', 'latex');
xlabel('$r/R$', 'Interpreter', 'latex');
set(gca, 'TickLabelInterpreter', 'latex');
xlim(gca, [0, 1]);

if savefigs
    fpath = fullfile(plot_fld, 'T5_dC_T_vs_r.pdf');
    exportgraphics(gcf, fpath, 'ContentType', 'vector', ...
        'BackgroundColor', 'none', 'Resolution', 300);
end

%% Plot dP
figure(fig_index);
fig_index = fig_index + 1;
resizeFigure(gcf, 800, 400);
cla; hold on; grid on;

plot(r/R, squeeze(res_opt.dP), LineWidth=lw(1), LineStyle='-', Color='k')

set(gcf,'Color','White');
set(gca,'FontSize',fs);
ylabel('$dP$ [W/m]', 'Interpreter', 'latex');
xlabel('$r/R$', 'Interpreter', 'latex');
set(gca, 'TickLabelInterpreter', 'latex');
xlim(gca, [0, 1]);

if savefigs
    fpath = fullfile(plot_fld, 'T5_dP_vs_r.pdf');
    exportgraphics(gcf, fpath, 'ContentType', 'vector', ...
        'BackgroundColor', 'none', 'Resolution', 300);
end

function resizeFigure(fig, width, height)
%resizeFigure Resizes the current figure to specified dimensions.
%   resizeFigure(fig, width, height) resizes the figure to width x height pixels.
    oldUnits = get(fig, 'Units');   % Store original units
    set(fig, 'Units', 'pixels');        % Temporarily change units

    pos = get(fig, 'Position');     % Get current position
    pos(3) = width;                 % Set new width
    pos(4) = height;                % Set new height
    set(fig, 'Position', pos);     % Apply new size

    set(fig, 'Units', oldUnits);   % Restore original units
end