classdef ShapeOptimizer
    properties
        % Ingenuity values
        omega_ing = 2800 * 2*pi / 60;  % Rotational speed [rad/s]
        R_ing = .6;  % Propeller radius of Ingenuity [m]

        % Constants
        g = 3.728;  % Gravity on Mars [m/s^2]
        rho = 14e-3;  % Atmosphere density on Mars [kg/m^3]

        % Airfoil coefficients
        alpha
        C_l
        C_d

        % Results from Task 2
        N_prop  % Number of propellers
        N_bld  % Number of blades per rotor
        R  % Rotor radius [m].
        m_tot  % Total mass of the drone (incl. payload) [kg]

        % Optimization input params
        A_rotor  % Area of one propeller
        A_prop  % Total area of all propellers
        T_req  % Total required thrust of the drone  (incl. payload) [N]
        omega  % Rotational speed [rad/s]
        C_T_req  % Required thrust coefficient.

        alpha_D
        c_tip  % Chord at the wing tip [m].
        c_root  % Chord at the wing root [m].
        r_thres  % Threshold after which the optimal chord distribution 
                     % should be used. Given as a ratio on the blade length.
        r_tip  % Threshold after which the wing tip chord should 
                   % smoothly transition to zero. Given as a ratio on the 
                   % blade length.
    end

    methods
        function obj = ShapeOptimizer(N_prop, N_bld, R, m_tot)
            % Airfoil coefficients
            [alpha, C_l, C_d] = load_airfoil_data('AG14_14k.txt');
            obj.alpha = alpha;
            obj.C_l = C_l;
            obj.C_d = C_d;
    
            % Results from Task 2
            obj.N_prop = N_prop;
            obj.N_bld = N_bld;
            obj.R = R;
            obj.m_tot = m_tot;
    
            % Optimization input params
            obj.A_rotor = pi .* R.^2;
            obj.A_prop = N_prop .* obj.A_rotor;
            obj.T_req = m_tot*obj.g;
            obj.omega = obj.omega_ing .* obj.R_ing ./ R;
            obj.C_T_req = obj.T_req / (0.5*obj.rho*obj.A_prop*(obj.omega*R)^2);
        end

        function c = generate_chord_distribution(obj, r, c_tip, c_root, r_thres, r_tip)
            if nargin < 5
                r_tip = .97;
            end
            if nargin < 4
                r_thres = .3;
            end

            c_cut = 2.5 * c_tip;  % Chord at which to cut the peak to smoothen the shape

            % Since the optimal chord distribution increases rapidly close to the root,
            % it it ony used for r/R > r_thres
            c_opt = c_tip ./ r(r/obj.R>=r_thres);
            
            % Calculate chord near root 
            c_root = (c_opt(1)-c_root)/(obj.R*r_thres) * r(r/obj.R<r_thres) + c_root;
            
            % Combine the two curves
            c_raw = horzcat(c_root, c_opt);
            
            % Smoothen out the sharp peak between the two curves and the wing tip
            idx_valid = c_raw<=c_cut & r./obj.R<=r_tip;
            r_valid = r(idx_valid);
            c_valid = c_raw(idx_valid);
            if r_tip<1
                r_valid(end+1) = obj.R;
                c_valid(end+1) = 0;
            end
            
            c = interp1(r_valid, c_valid, r, 'spline');
        end

        function theta = generate_twist_distribution(obj, r, alpha_D, r_thres)
            if nargin < 4
                r_thres = .3;
            end

            theta_opt = alpha_D + rad2deg(.5./r(r/obj.R>=r_thres) .* sqrt(obj.C_T_req));

            % Calculate chord near root
            theta_root = zeros(1,numel(r(r/obj.R<r_thres))) + theta_opt(1);
            
            %Combine the two curves
            theta_raw = horzcat(theta_root, theta_opt);
            
            % Smoothen out the sharp corner between the two curves
            idx_valid = r<=(r_thres-.05) | r>=(r_thres+.05);
            r_valid = r(idx_valid);
            theta_valid = theta_raw(idx_valid);
            
            theta = interp1(r_valid, theta_valid, r, 'spline');
        end

        function [res_opt, res] = optimize(obj, r, alpha_D, c_tip, c_root, r_thres, r_tip)
            if nargin < 6
                r_tip = .97;
            end
            if nargin < 5
                r_thres = .3;
            end
            res_opt = struct();
            res = struct();

            % Preallocate arrays
            N_r = numel(r); N_c = numel(c_tip); N_alpha = numel(alpha_D);

            res.r = r;
            res.c_tip = c_tip;
            res.c_root = c_root;
            res.r_thres = r_thres;
            res.r_tip = r_tip;
            res.alpha_D = alpha_D;
            res.c = zeros(N_c, N_alpha, N_r);
            res.theta = zeros(N_c, N_alpha, N_r);
            res.dT = zeros(N_c, N_alpha, N_r);
            res.dP = zeros(N_c, N_alpha, N_r);
            res.n_it = zeros(N_c, N_alpha, N_r);
            res.P = zeros(N_c, N_alpha);
            res.T = zeros(N_c, N_alpha);
            for i = 1:numel(c_tip)
                for j = 1:numel(alpha_D)
                    c = obj.generate_chord_distribution(r, c_tip(i), ...
                        c_root(i), r_thres, r_tip);
                    theta = obj.generate_twist_distribution(r, ...
                        alpha_D(j), r_thres);
                    res.c(i,j,:) = c;
                    res.theta(i,j,:) = theta;

                    v_h = sqrt(obj.m_tot / (2 * obj.rho * obj.A_rotor));  % Hover velocity [m]

                    blade = RotorBlade(r, c, deg2rad(theta), obj.alpha, ...
                        obj.C_l, obj.C_d);
                    bem = BEM(blade, obj.rho);

                    [bem, P, T] = bem.solve(obj.omega, obj.N_bld, 0, v_h);
                    bem_res_int = bem.res_int;
                    bem_res = bem.res;
                    
                    res.n_it(i,j,:) = sum(bem_res_int.dT_diff ~= 0, 1);
                    res.dP(i,j,:) = bem_res.dP;
                    res.dT(i,j,:) = bem_res.dT_BE;

                    P_total = obj.N_prop .* P;
                    T_total = obj.N_prop .* T;

                    res.P(i,j) = P_total;
                    res.T(i,j) = T_total;
                end
            end

            % Find optimal design
            % Logical mask of where T >= T_req
            valid_idx = res.T >= obj.T_req;
            
            % Set invalid power entries to Inf so they won't be chosen as minimum
            P_filtered = res.P;
            P_filtered(~valid_idx) = Inf;
            
            % Find the index of the minimum power in the filtered array
            [minP, linear_idx] = min(P_filtered(:));
            idx_c = mod(linear_idx, numel(c_tip));
            idx_alpha = ceil(linear_idx/numel(c_tip));

            % Extract optimal design values
            res_opt.c_tip = c_tip(idx_c);
            res_opt.c_root = c_root(idx_c);
            res_opt.r_thres = r_thres;
            res_opt.r_tip = r_tip;
            res_opt.alpha_D = alpha_D(idx_alpha);
            res_opt.P = res.P(idx_c, idx_alpha);
            res_opt.T = res.T(idx_c, idx_alpha);
            res_opt.dT = res.dT(idx_c, idx_alpha, :);
            res_opt.dP = res.dP(idx_c, idx_alpha, :);
            res_opt.c = res.c(idx_c, idx_alpha, :);
            res_opt.theta = res.theta(idx_c, idx_alpha, :);
        end
    end
end