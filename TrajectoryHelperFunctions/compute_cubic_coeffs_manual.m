function [cubic_coeffs_s1,cubic_coeffs_s2]= compute_cubic_coeffs_manual(xi,vxi,xm,vxm,xf,vxf,tf,tm)
% Calculates the coefficients of a trajectory with one intermediate point
% in manual mode

B = [xi; vxi; xm; xm; vxm; vxm; xf; vxf];

A = [1 0 0 0 0 0 0 0;
     0 1 0 0 0 0 0 0;
     1 tm tm^2 tm^3 0 0 0 0;
     0 0 0 0 1 0 0 0;
     0 1 2*tm 3*tm^2 0 0 0 0;
     0 0 0 0 0 1 0 0;
     0 0 0 0 1 (tf-tm) (tf-tm)^2 (tf-tm)^3;
     0 0 0 0 0 1 2*(tf-tm) 3*(tf-tm)^2;
    ];

cubic_coeffs = transpose(A\B);
cubic_coeffs_s1 = cubic_coeffs(1:4);
cubic_coeffs_s2 = cubic_coeffs(5:8);
end