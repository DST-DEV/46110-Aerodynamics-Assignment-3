clear; clc; close all;

%% Preparations
R = 1;  % Rotor radius [m]
alpha_D = 5;  % Design angle of attack [°]
C_T = 3.45;  % Thrust coefficient
c_tip = .07;  % Chord at the wind tip [m]

r = 0:.01:R;  % spanwise coordinates [m]

%% Calculate chord distribution
% Since the optimal chord distribution increases rapidly close to the root,
% it it ony used for r/R > r_thres
r_thres = .3;  % Threshold after which the optimal chord distribution should be used
c_opt = c_tip ./ r(r/R>=r_thres);

% Calculate chord near root 
c_root = c_opt(1)/(R*r_thres) * r(r/R<r_thres);

%Combine the two curves
c = horzcat(c_root, c_opt);

figure(1);
plot(r, c);

%% Calculate twist distribution
theta_opt = alpha_D + .5./r(r/R>=r_thres) .* sqrt(C_T);

% Calculate chord near root
theta_root = theta_opt(1)/(R*r_thres) * r(r/R<r_thres);

%Combine the two curves
theta = horzcat(theta_root, theta_opt);

% theta_opt_2 = 2*C_T./(sigma.*C_la) + .5*sqrt(C_T) + 2/3*alpha_0 

figure(2);
plot(r, theta);