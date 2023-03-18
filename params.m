%system params
g = 9.81;
m = 1;
e3 = [0;0;1];
J = diag([1 1 1]);
orden = 0.5;

%position gains
K1 = diag([50 50 50]);
K1_pid = diag([70 70 70]);
K2 = diag([10 10 10]);
K2_pid = diag([20 20 20]);
K3_pid = diag([1 1 1]);
A = diag([7 7 7]);

%attitude gains
k3 = diag([30 30 30]);
k4 = diag([10 10 10]);
kR = diag([50 50 50]);
komega = diag([10 10 10]);