classdef RotorBlade
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here

    properties
        r
        dr
        R
        c
        theta
        aoa
        C_l
        C_d

    end

    methods
        function obj = RotorBlade(r, c, theta, aoa, C_l, C_d)
            obj.r = r;
            % obj.dr = zeros(1, numel(r))
            obj.dr(1) = r(1);
            for i = 2:numel(r)
                obj.dr(i) = r(i)-r(i-1);
            end
            obj.R = r(end);

            obj.c = c;
            obj.theta = theta;

            obj.aoa = aoa;
            obj.C_l = C_l;
            obj.C_d = C_d;
        end

        function [C_l, C_d] = aero_coeffs(obj, aoa)
            C_l = interp1(obj.aoa, obj.C_l, aoa,'spline','extrap');
            C_d = interp1(obj.aoa, obj.C_d, aoa,'spline','extrap');
        end
    end
end