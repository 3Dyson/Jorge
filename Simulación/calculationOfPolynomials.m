%% CALCULATION OF POLYNOMIALS

clear all, close all, clc

tick = 1e-3; % granularity
tTop = 20e-3; % max sampling period

A = [0 -23.8095; 0 0];
B = [0; -23.8095];
C = [1 0];
D = [0];

sys = ss(A, B, C, D);

n = 1;
for i = 1e-3 : tick : tTop 
sys_d = c2d(sys, i);
A1(n) = sys_d.a(1,1);
A2(n) = sys_d.a(1,2);
A3(n) = sys_d.a(2,1);
A4(n) = sys_d.a(2,2);
B1_1(n) = sys_d.b(1,1);
B2_1(n) = sys_d.b(2,1);
x(n)= i;
n = n+1;
end

%% Optimal design
Q = [2 0; 0 0.5];
R = 0.01;
m = 1;
for i = 1e-3 : tick : tTop % gains
[K,S,e] = lqrd(A,B,Q,R,i);
K1_1(m) = K(1,1);
K1_2(m) = K(1,2);
m = m+1;
end

%% polyfit A(1,2)
p1 = polyfit(x, A2, 1)
g1 = polyval(p1,x); 
figure
plot(x, A2,'ro', x, g1 ,'b'); 
title('Valor de A(1,2)');
legend('A(1,2)', 'Ajuste polinómico');
xlabel('τ_k (sec)');
ylabel('a(1,2)');

%% polyfit B(1,1)
p2 = polyfit(x, B1_1, 2)
g2 = polyval(p2,x);
figure
plot(x, B1_1,'ro', x, g2 ,'b'); 
title('Valor de B(1,1)');
legend('B(1,1)', 'Ajuste polinómico');
xlabel('τ_k (sec)');
ylabel('b(1,1)');

%% polyfit B(2,1)
p3 = polyfit(x, B2_1, 1)
g3 = polyval(p3,x);
figure
plot(x, B2_1,'rO', x, g3 ,'b'); 
title('Valor de B(2,1)');
legend('B(2,1)', 'Ajuste polinómico');
xlabel('τ_k (sec)');
ylabel('b(2,1)');

%% polyfit K(1,1)
p4 = polyfit(x, K1_1, 4)
g4 = polyval(p4,x);
figure
plot(x, K1_1,'ro', x, g4 ,'b'); 
title('Valor de K(1,1)');
legend('K(1,1)', 'Ajuste polinómico');
xlabel('τ_k (sec)');
ylabel('k(1,1)');
 
%% polyfit K(1,2)
p5 = polyfit(x, K1_2, 4)
g5 = polyval(p5,x);
figure
plot(x, K1_2,'ro', x, g5 ,'b'); 
title('Valor K(1,2)');
legend('K(1,2)', 'Ajuste polinómico');
xlabel('τ_k (sec)');
ylabel('K(1,2)');

%% gains graph
p4 = polyfit(x, K1_1, 4)
g4 = polyval(p4,x);
p5 = polyfit(x, K1_2, 4)
g5 = polyval(p5,x);
figure
plot(x, K1_1,'ro', x, g4 ,'r',x,K1_2,'bo',x,g5); 
title('Ganancias del controlador');
legend('K11(τ_k)', 'Ajuste K11(τ_k)','K21(τ_k)', 'Ajuste K21(τ_k)');
xlabel('τ_k (sec)');
ylabel('K(τ_k)');
  