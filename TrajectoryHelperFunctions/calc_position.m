function x = calc_position(coeffs,t)
% Polynomial equation
% x(t) = a0 + a1*t + a2*t^2 + a3*t^3

a0 = coeffs(1);
a1 = coeffs(2);
a2 = coeffs(3);
a3 = coeffs(4);

x = a0 + a1*t + a2*t^2 + a3*t^3;
end