clear; clc; close all;
%% Preparations
R = 1;  % Rotor radius [m]
alpha_D = 5;  % Design angle of attack [°]
C_T = 3.45;  % Thrust coefficient
c_tip = .1;  % Chord at the wind tip [m]

r = 0:.01:R;  % spanwise coordinates [m]

%% Calculate chord distribution
% Since the optimal chord distribution increases rapidly close to the root,
% it it ony used for r/R > r_thres
r_thres = .95;  % Threshold after which the optimal chord distribution should be used
c_opt = c_tip ./ r(r/R>=r_thres);

% Calculate chord near root by using the slope of the optimal chord
% distribution at r/R = r_thres
slope = (c_opt(2) - c_opt(1)) / (r(2) - r(1));
r_root = r(r/R<=r_thres);
c_root = slope*(r_root - r_root(end)) + c_opt(1);

%Combine the two curves
c = horzcat(c_root(1:end-1), c_opt);

figure(1);
plot(r, c);

%% Calculate twist distribution
theta_opt = alpha_D + .5./r .* sqrt(C_T);
% theta_opt_2 = 2*C_T./(sigma.*C_la) + .5*sqrt(C_T) + 2/3*alpha_0 

figure(2);
plot(r, theta_opt);