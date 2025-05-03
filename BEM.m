classdef BEM
    properties
        blade  % Rotor blade
        rho  % Air density [kg/m^3]
        res_int  % Intermediate results structure array
        res  % Results structure array
    end

    methods
        function obj = BEM(blade, rho)
            obj.blade = blade;
            obj.rho = rho;
            obj.res_int = struct();
            obj.res = struct();
        end

        function [obj, P, T] = solve(obj, omega, N_b, V_c, v_i_0)
            if nargin < 4
                V_c = 0;
            end
            if nargin < 5
                v_i_0 = 0;
            end
            
            max_iter = 2000;

            % Get wing geometry
            idx_valid = obj.blade.r<.999*obj.blade.R & obj.blade.r>0;
            % idx_valid = obj.blade.r<.99*obj.blade.R & obj.blade.r>.02*obj.blade.R;
            idx_valid = 1:numel(obj.blade.r);
            r = obj.blade.r(idx_valid);
            dr = obj.blade.dr(idx_valid);
            c = obj.blade.c(idx_valid);
            theta = obj.blade.theta(idx_valid);

            % Preallocate arrays
            obj = obj.init_res(r);
            obj.res.r = r;
            obj.res.c = c;
            obj.res.dr = dr;
            obj = obj.init_res_int(r, max_iter);
            
            for i = 1:numel(r)
                v_i = v_i_0;  % Initalize induced wind
                % v_i = 0;  % Initalize induced wind
                converged = false;
                n_it = 1;
                while ~converged && n_it < max_iter
                    V_rel = sqrt((V_c+v_i).^2 + (omega.*r(i)).^2);
                    phi = acos((omega.*r(i))./V_rel);
                    aoa = theta(i) - phi;
                    
                    % Save results
                    obj.res_int.V_rel(n_it, i) = V_rel; 
                    obj.res_int.phi(n_it, i) = phi; 
                    obj.res_int.aoa(n_it, i) = aoa; 

                    [C_l, C_d] = obj.blade.aero_coeffs(rad2deg(aoa));

                    % Calculate normal and tangential force factors
                    C_n = C_l*cos(phi) - C_d*sin(phi);
                    C_t = C_l*sin(phi) + C_d*cos(phi);

                    % Save results
                    obj.res_int.C_l(n_it, i) = C_l; 
                    obj.res_int.C_d(n_it, i) = C_d; 
                    obj.res_int.C_n(n_it, i) = C_n; 
                    obj.res_int.C_t(n_it, i) = C_t; 

                    % Calculate Prandtl correction factor
                    f = (N_b / 2) ...
                        .* (obj.blade.R - r(i)) / (r(i) * abs(sin(phi)));  % Eq. 89
                    if isfinite(f) && f > 1e-6
                        F = (2 / pi) * acos(exp(-f)); % Eq. 89
                    else
                        F = 1;  % Tip loss negligible or invalid f
                    end

                    % Save results
                    obj.res_int.f(n_it, i) = f;
                    obj.res_int.F(n_it, i) = F;

                    % Calculate local thrust
                    dT_BE = .5.*N_b.*obj.rho.*c(i)...
                        .*(V_rel.^2).*C_n.*dr(i);  % Eq. 88

                    dT_mom = 4*pi*obj.rho...
                        .*(V_c + v_i).*v_i.*r(i).*dr(i);  % Eq. 87
                    
                    v_i_old = v_i;
                    v_i = v_i + .4*(dT_BE - F.*dT_mom);  % Eq. 90

                    % Save results
                    obj.res_int.dT_BE(n_it, i) = dT_BE;
                    obj.res_int.dT_mom(n_it, i) = dT_mom;
                    obj.res_int.v_i(n_it, i) = v_i;
                    
                    % dT_diff = abs(dT_BE - F.*dT_mom);
                    % if dT_diff <1e-4
                    %     converged = true;
                    % end
                    dT_diff = abs(dT_BE - F.*dT_mom);
                    if abs(v_i-v_i_old) <1e-5
                        converged = true;
                    end
                    obj.res_int.dT_diff(n_it, i) = dT_diff;

                    n_it = n_it + 1;
                end

                if n_it == max_iter
                    fprintf('Warning: BEM did not converge for radius r = %.3f\n', r(i));
                end

                % Save results from last iteration
                obj.res.V_rel(i) = V_rel; 
                obj.res.phi(i) = phi; 
                obj.res.aoa(i) = aoa; 
                obj.res.C_l(i) = C_l; 
                obj.res.C_d(i) = C_d; 
                obj.res.C_n(i) = C_n; 
                obj.res.C_t(i) = C_t;
                obj.res.f(i) = f;
                obj.res.F(i) = F;
                obj.res.dT_BE(i) = dT_BE;
                obj.res.dT_mom(i) = dT_mom;
                obj.res.v_i(i) = v_i;
                obj.res.dT_diff(i) = dT_diff;
            end

            % Calculate total thrust and power of the rotor
            dM = .5.*N_b.*obj.rho.*c.*(obj.res.V_rel.^2).*obj.res.C_t.*dr.*r;
            dP = dM*omega;
            
            % % Insert 0 at the upper and lower end of the radii ranges
            % r_full = [0, r, obj.blade.R];
            % dM = [0, dM, 0];
            % dP = [0, dP, 0];
            % dT_BE = [0, obj.res.dT_BE, 0];
            % dT_mom = [0, obj.res.dT_mom, 0];
            r_full = r;
            dT_BE = obj.res.dT_BE;
            dT_mom = obj.res.dT_mom;


            % Replace NaN values
            dP(isnan(dP))=0;
            dM(isnan(dM))=0;
            dT_BE(isnan(dT_BE))=0;
            dT_mom(isnan(dT_mom))=0;

            % Save results
            obj.res.r_full = r_full;
            obj.res.dM = dM;
            obj.res.dP = dP;
            obj.res.dT_BE = dT_BE;
            obj.res.dT_mom = dT_mom;
            
            P = sum(dP);
            T = sum(dT_BE);

            % Convert angles to degrees
            obj.res.aoa = rad2deg(obj.res.aoa);
            obj.res.phi = rad2deg(obj.res.phi);
            obj.res_int.aoa = rad2deg(obj.res_int.aoa);
            obj.res_int.phi = rad2deg(obj.res_int.phi);
        end

        function obj = init_res_int(obj, r, max_iter)
            obj.res_int.n = 1:max_iter-1;

            init_arr = zeros(max_iter, numel(r));
            obj.res_int.v_i = init_arr;
            obj.res_int.V_rel = init_arr;
            obj.res_int.phi = init_arr;
            obj.res_int.aoa = init_arr;
            obj.res_int.C_d = init_arr;
            obj.res_int.C_n = init_arr;
            obj.res_int.C_t = init_arr;
            obj.res_int.f = init_arr;
            obj.res_int.F = init_arr;
            obj.res_int.dT_BE = init_arr;
            obj.res_int.dT_mom = init_arr;
            obj.res_int.dT_diff = init_arr;
        end

        function obj = init_res(obj, r)
            init_arr = zeros(1, numel(r));
            obj.res.v_i = init_arr;
            obj.res.V_rel = init_arr;
            obj.res.phi = init_arr;
            obj.res.aoa = init_arr;
            obj.res.C_d = init_arr;
            obj.res.C_n = init_arr;
            obj.res.C_t = init_arr;
            obj.res.f = init_arr;
            obj.res.F = init_arr;
            obj.res.dT_BE = init_arr;
            obj.res.dT_mom = init_arr;
            obj.res.dT_diff = init_arr;
        end
    end
end