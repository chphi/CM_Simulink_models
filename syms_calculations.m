%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Computes the analytical expression of "alpha_1" for the kinematic
% controller in chained form in [Snider2009]
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc
clear

syms t
syms L
syms e_y
syms theta_e
syms delta
syms pcoeff

syms ksi(s)

% definition of the state x2
x_2(s, e_y, theta_e, delta) = ...
    ksi(s)*(1-e_y*ksi(s))*(1+sin(theta_e)^2)/(cos(theta_e)^2) ...
    + (1-e_y*ksi(s))^2 * tan(delta) / (L*cos(theta_e)^3); ...


% derivative of x_2 with respect to s
dx2_ds = diff(x_2,s);

% forces d(ksi)/d(s) to zero (difficult to measure so better not bother
% with it)
dksi_ds = diff(ksi,s);
dx2_ds = subs(dx2_ds, dksi_ds, 0);

% derivative of x_2 with respect to e_y
dx2_dera = diff(x_2,e_y);

% derivative of x_2 with respect to theta_e
dx2_dthetae = diff(x_2, theta_e);

% coefficient alpha_1 of the chained form in [Snider2009] :
alpha_1(s, e_y, theta_e, delta) = ...
      dx2_ds(s, e_y, theta_e, delta) ...
    + dx2_dera(s, e_y, theta_e, delta) * (1-e_y*ksi) * tan(theta_e) ...
    + dx2_dthetae(s, e_y, theta_e, delta) ...
        * ( tan(delta)*(1-e_y*ksi)/(L*cos(theta_e)) - ksi );

alpha_1 = simplify(alpha_1);






