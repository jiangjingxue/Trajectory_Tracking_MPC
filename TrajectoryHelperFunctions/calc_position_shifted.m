function x = calc_position_shifted(coeffs,t,tm)
% Polynomial equation
% x(t) = a0 + a1*t + a2*t^2 + a3*t^3
 
a0 = coeffs(1);
a1 = coeffs(2);
a2 = coeffs(3);
a3 = coeffs(4);

x = a0 + a1*(t-tm) + a2*(t-tm)^2 + a3*(t-tm)^3;
end