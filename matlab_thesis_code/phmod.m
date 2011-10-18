function M = phmod(x)
% Calculates phase angle mod 2*pi

M = mod(x+pi,2*pi)-pi;