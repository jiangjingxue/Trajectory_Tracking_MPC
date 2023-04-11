function acc = calc_acceleration(coeffs,t)
% Polynomial equation
% a(t) = 2*a2 + 6*a3*t

a2 = coeffs(3);
a3 = coeffs(4);

acc = 2*a2 + 6*a3*t;
end