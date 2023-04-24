s = tf('s');
G = 1030/s;
D = (s+0.1)/s*9.4406e-04
[num,den] = tfdata(D)
D2 = 0.001*(s+1)/s;
[num2,den2] = tfdata(D2)
bode(G*D)