clear all
clc
syms g mw mf Jw Jf lw lf thetaw thetaf thetawf Bw Bf tau dthetaf dthetaw dthetawf ddthetaw ddthetaf ddthetawf


T = (Jf+mw*lw^2)*(dthetaf)^2/2+Jw*dthetawf^2/2;
% Kinetic energy comes from both frame and wheel turning (note that wheel
% is assumed to be a point mass) as wheel as the relative turning speed of
% the wheel
U = (mw*lw+mf*lf)*g*cos(thetaf);
% Potential energy comes from the gravitational potential energy of both 
% elements
D = 1/2*Bf*dthetaf^2 + 1/2*Bw*(dthetawf-dthetaf)^2;
% System losses come from the dynamic friction of the overall system and
% the dynamic friction on the relative speed of the wheel
Q = [tau; -tau];

% Get lagrange eqs
L = T-U;

equ2 = solve(subs(diff(L,dthetawf), [dthetawf dthetaf],[ddthetawf,ddthetaf]) == diff(L,thetawf)-diff(D,dthetawf)+Q(1),ddthetawf);
equ1 = solve(subs(diff(L,dthetaf), [dthetawf dthetaf thetawf thetaf],[ddthetawf,ddthetaf, dthetawf, dthetaf]) == diff(L,thetaf)-diff(D,dthetaf)+Q(2),ddthetaf);

% Get it in thetaw and thetaf form
equ1 = simplify(subs(equ1, [ddthetawf dthetawf thetawf],[ddthetaw+ddthetaf , dthetaw+dthetaf, thetaw+thetaf]))
equ2 = simplify(subs(equ2, [ddthetawf dthetawf thetawf],[ddthetaw+ddthetaf , dthetaw+dthetaf, thetaw+thetaf]))-ddthetaf;

%Linearize around thetaf = 0, thetaw = 0, dthetaf = 0
equ1 = subs(equ1,sin(thetaf),thetaf);

% Solve for ddthetaf and ddtehtaw;

solve_theta2 = solve(equ2==ddthetaw,ddthetaw);
solve_theta1 = simplify(solve(subs(equ1,ddthetaw,solve_theta2) == ddthetaf,ddthetaf));
solve_theta2 = simplify(solve(subs(equ2,ddthetaf,solve_theta1) == ddthetaw,ddthetaw));

% Express system in
A1 =coeffs(solve_theta1,[thetaf,dthetaf,dthetaw,tau]);
A2 =coeffs(solve_theta2,[thetaf,dthetaf,dthetaw,tau]);
A = [ 0 1 0 0;
    flip(A1);
     flip(A2)];
B = A(:,4)
A = A(:,1:3)
X = [thetaf;dthetaf;thetaw]
U = [tau]

% Obtain laplace equation:
syms s
L1 = subs(solve_theta1,[dthetaf dthetaw],[s*thetaf s*thetaw]) == thetaf*s^2;
L2 = subs(solve_theta2,[dthetaf dthetaw],[s*thetaf s*thetaw]) == thetaw*s^2;

aux = solve(L2,thetaw);
L1 = simplify(solve(subs(L1,thetaw,aux),thetaf))

% Obtain numeraton and denominator
[Num Den] = numden(L1);

Num = coeffs(Num,s,'All')
Den = coeffs(Den,s,'All')

g_val  = 9.81;
mw_val = 0.22;
mf_val = 0.546;
lw_val = 0.092;
lf_val = 0.0958;
Jw_val = 0.699e-3;
Jf_val = 0.0067;
Bw_val = 1.93e-5;
Bf_val = 0.0036;

Num  = subs(Num,[g mw mf lw lf Jw Jf Bf Bw tau],[g_val mw_val mf_val lw_val lf_val Jw_val Jf_val Bf_val Bw_val 1])
Den  = subs(Den,[g mw mf lw lf Jw Jf Bf Bw tau],[g_val mw_val mf_val lw_val lf_val Jw_val Jf_val Bf_val Bw_val 1])

Num = double(Num);
Den = double(Den);

Num = Num/Den(1);
Den = Den/Den(1);

G = tf(Num,Den)

