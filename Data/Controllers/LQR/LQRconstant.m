function [] = LQRkonstanter(inputArg1,inputArg2,inputArg3)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
s=tf('s');
l = 0.092;
m_R = 0.412;
J_R = 8.9*10^-3;
B_R = 2.2*10^-3;
m_H = 0.220;
J_H = 0.694*10^-3;
B_H = 0.199*10^-6;
g = 9.82;

p1 = (-B_R) / (J_R-m_H*l^2);
p2 = B_H / (J_R-m_H*l^2);
p3 = (l*(m_R+m_H)*g) / (J_R-m_H*l^2);
p4 = 1 / (J_R-m_H*l^2);
p5 = 1 / J_H;
p6 = B_H/J_H;

A = [0 1 0 ; p3 p1 p2 ; -p3 -p1 -(p6+p2)];
B = [0 ; -p4 ; p5+p4];
C = [1 0 0 ; 0 1 0 ; 0 0 1];
D = zeros(3,1);

tt = A + .5*eye(3);

%Testing to see if eig works
v = [-6.2405 + 2.3051i;-6.2405 - 2.3051i;-1.3202 + 0.0000i];
k = place(A, B, v);
Acl = A-B*k;

%Actual LQR
Q = [1.62*inputArg1 0 0;
     0 13*10^-3 0;
     0 0 2.53*10^-6];
R = 15.84*inputArg2;
[k, S, e] = lqr(A, B, Q, R);
inputArg3
k;
k2 = [-1.1536,-0.1998,-0.0015];
Ac2 = A-B*k2;
eig(Acl);
eig(Ac2)
G = ss(A-B*k,B,C,D);
SettleMax=1*0.05;
SettleMin=-1*0.05;

figure()
initial(G(1,1),[-1 0 0])
grid on

figure()
initial(G(1,1),[-1 0 0])
ylim([-0.1 0.1])
yline(0)
yline(SettleMax,'--')
yline(SettleMin,'--')
grid on
end
