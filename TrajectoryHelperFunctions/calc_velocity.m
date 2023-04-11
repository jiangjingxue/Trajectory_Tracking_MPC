function v = calc_velocity(coeffs,t)
% Polynomial equation
% v(t) = a1 + 2*a2*t + 3*a3*t^2

a1 = coeffs(2);
a2 = coeffs(3);
a3 = coeffs(4);

v = a1 + 2*a2*t + 3*a3*t^2;
end