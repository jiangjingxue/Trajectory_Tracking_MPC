function v = calc_velocity_shifted(coeffs,t,tm)
% Polynomial equation
% v(t) = a1 + 2*a2*t + 3*a3*t^2

a1 = coeffs(2);
a2 = coeffs(3);
a3 = coeffs(4);

v = a1 + 2*a2*(t-tm) + 3*a3*(t-tm)^2;
end