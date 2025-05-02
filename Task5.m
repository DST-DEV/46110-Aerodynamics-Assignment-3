clear; clc; close all;

savefigs = true;
res_fld = 'results';
plot_fld = 'plots';

%% Plot settings
cols = ["#0072BD", "#D95319", "#EDB120", "#77AC30", "#80B3FF"];  % Colors of the lines
lw = [1, 1, 1, 1, 1];  % Linewidth for the lines of the four methods
ax_col = [0.2, 0.2, 0.2];  % Color of accented axes
ax_lw = 1.5;  % Line width of accented axes
fs = 16;  % Plot font size

%% Results from previous taks

% Ingenuity values
omega_ing = 2800 * 2*pi / 60;  % Rotational speed [rad/s]
R_ing = .6;  % Propeller radius of Ingenuity [m]

% Constants
g = 3.728;  % Gravity on Mars [m/s^2]
rho = 14e-3;  % Atmosphere density on Mars [kg/m^3]

% Task 2
N_prop = 4;  % Number of propellers
N_bld = 2;  % Number of blades per rotor
R = 1;  % Rotor radius [m].
A_prop = N_prop .* pi .* R.^2;  % Total area of all propellers
m_tot = 4.846;  % Total mass of the drone (incl. payload) [kg]
T_req = m_tot*g;  % Total required thrust of the drone  (incl. payload) [N]
omega = omega_ing .* R_ing ./ R;  % Rotational speed

C_T_req = T_req / (0.5*rho*A_prop*(omega*R)^2);  % Thrust coefficient.

%% Preparations

alpha_D = 5;  % Design angle of attack [°].
c_tip = .07;  % Chord at the wing tip [m].
c_root = .08;  % Chord at the wing root [m].
r_thres = .3;  % Threshold after which the optimal chord distribution 
               % should be used. Given as a ratio on the blade length.
c_cut = .18;  % Upper limit for the chord, at which the peak should be cut 
              % and smoothened.
r_tip = .97;  % Threshold after which the wing tip chord should smoothly 
              % transition to zero. Given as a ratio on the blade length.

r = 0:.01:R;

%% Calculate chord distribution
% Since the optimal chord distribution increases rapidly close to the root,
% it it ony used for r/R > r_thres
c_opt = c_tip ./ r(r/R>=r_thres);

% Calculate chord near root 
c_root = (c_opt(1)-c_root)/(R*r_thres) * r(r/R<r_thres) + c_root;

% Combine the two curves
c = horzcat(c_root, c_opt);

% Smoothen out the sharp peak between the two curves and the wing tip
idx_valid = c<=c_cut & r./R<=r_tip;
r_valid = r(idx_valid);
c_valid = c(idx_valid);
if r_tip<1
    r_valid(end+1) = R;
    c_valid(end+1) = 0;
end

c_full = interp1(r_valid, c_valid, r, 'spline');

% Plot wing shape
fig_index = 1;
figure(fig_index);
fig_index = fig_index + 1;
resizeFigure(gcf, 800, 400);

cla; hold on; grid on;
plot (r/R, c_tip ./ r, LineWidth=lw(1), LineStyle='--', Color='k', ...
    DisplayName='Ideal chord');
plot(r, c, LineWidth=lw(2)*1.5, LineStyle='-.', Color=cols(2), ...
    DisplayName='Raw chord distribution'); 
plot (r_valid, c_valid, LineWidth=lw(3)*1.5, LineStyle='-.', Color=cols(3), ...
    DisplayName='Filtered chord distribution');
plot (r/R, c_full, LineWidth=lw(1), LineStyle='-', Color='k', ...
    DisplayName='Final chord distribution');
hold off;

set(gcf,'Color','White');
set(gca,'FontSize',fs);
ylabel('$c$ [m]', 'Interpreter', 'latex');
xlabel('$r/R$', 'Interpreter', 'latex');
set(gca, 'TickLabelInterpreter', 'latex');
xlim(gca, [0, 1]);
ylim(gca, [0, max(c_full) + .05]);
legend('Interpreter', 'latex', 'Location', 'northeast');

if savefigs
    fpath = fullfile(plot_fld, 'T5_c_vs_r.pdf');
    exportgraphics(gcf, fpath, 'ContentType', 'vector', ...
        'BackgroundColor', 'none', 'Resolution', 300);
end

%% Calculate twist distribution
theta_opt = alpha_D + rad2deg(.5./r(r/R>=r_thres) .* sqrt(C_T_req));

% Calculate chord near root
theta_root = zeros(1,numel(r(r/R<r_thres))) + theta_opt(1);

%Combine the two curves
theta = horzcat(theta_root, theta_opt);

% Smoothen out the sharp corner between the two curves
idx_valid = r<=(r_thres-.05) | r>=(r_thres+.05);
r_valid = r(idx_valid);
theta_valid = theta(idx_valid);

theta_full = interp1(r_valid, theta_valid, r, 'spline');


% theta_opt_2 = 2*C_T./(sigma.*C_la) + .5*sqrt(C_T) + 2/3*alpha_0 

% Plot twist distribution
figure(fig_index);
fig_index = fig_index + 1;
resizeFigure(gcf, 800, 400);

cla; hold on; grid on;
plot (r/R, alpha_D + rad2deg(.5./r .* sqrt(C_T_req)), ...
    LineWidth=lw(1), LineStyle='--', Color='k', DisplayName='Ideal twist');
plot(r, theta, LineWidth=lw(2)*1.5, LineStyle='-.', Color=cols(2), ...
    DisplayName='Raw twist distribution'); 
plot (r_valid, theta_valid, LineWidth=lw(3)*1.5, LineStyle='-.', Color=cols(3), ...
    DisplayName='Filtered twist distribution');
plot(r/R, theta_full, LineWidth=lw(1), LineStyle='-', Color='k', ...
    DisplayName='Final twist distribution')
hold off;

set(gcf,'Color','White');
set(gca,'FontSize',fs);
ylabel('$\theta$ [$^{\circ}$]', 'Interpreter', 'latex');
xlabel('$r/R$', 'Interpreter', 'latex');
set(gca, 'TickLabelInterpreter', 'latex');
xlim(gca, [0, 1]);
ylim(gca, [min(theta)-1, max(theta) + 1]);
legend('Interpreter', 'latex', 'Location', 'northeast');

if savefigs
    fpath = fullfile(plot_fld, 'T5_theta_vs_r.pdf');
    exportgraphics(gcf, fpath, 'ContentType', 'vector', ...
        'BackgroundColor', 'none', 'Resolution', 300);
end

%% Solve BEM
% Airfoil coefficients
aoa = -180:5:180;
C_l = 2*pi*deg2rad(aoa+4);
C_d = zeros(1, numel(aoa));

% Create blade and BEM object
rho = 14e-3;  % Air density [kg/m^3]
blade = RotorBlade(r, c_full, deg2rad(theta_full), aoa, C_l, C_d);
bem = BEM(blade, rho);

% Estimate induced wind from momentum theory in hover
A_rotor = pi * blade.R.^2; % Rotor area [m^2]
v_h = sqrt(m_tot / (2 * bem.rho * A_rotor));  % Hover velocity [m] 

% Solve BEM
[bem, P, T] = bem.solve(omega, N_bld, 0, v_h);
res_int = bem.res_int;
res = bem.res;

P_total = N_prop .* P
T_total = N_prop .* T

if T_total < T_req
    disp('Warning: Insufficient thrust');
end

%% Plot BEM results
% Plot dC_T
dC_T = res.dT_BE ./ (.5*bem.rho.*A_rotor.*(omega.*blade.R).^2);

figure(fig_index);
fig_index = fig_index + 1;
resizeFigure(gcf, 800, 400);
cla; hold on; grid on;

plot(res.r_full/blade.R, dC_T, LineWidth=lw(1), LineStyle='-', Color='k')

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

% Plot dP
figure(fig_index);
fig_index = fig_index + 1;
resizeFigure(gcf, 800, 400);
cla; hold on; grid on;

plot(res.r_full/blade.R, res.dP, LineWidth=lw(1), LineStyle='-', Color='k')

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


% figure(3);
% set(gcf, 'Position', [100, 500, 800, 400]); grid on;
% plot(res_int.n, res_int.dT_mom(:,50)'); hold on
% plot(res_int.n, res_int.dT_BE(:,50)'); hold off

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

%% Export results
% res = struct('r', r, 'c', c_full, 'theta', theta_full, '');
% save(fullfile(res_fld, 'T5_res.mat'), 'res');