%% CONTROLLER DESIGN

clear all, close all, clc

global A B C D
h = 12e-3; %sampling period

A = [0 -23.8095; 0 0];
B = [0; -23.8095];
C = [1 0];
D = [0];

sys = ss(A, B, C, D)
sys_d = c2d(sys, h)

%% Optimal design
Q = [2 0; 0 0.5];
R = 0.01;
[K,S,e] = lqrd(A,B,Q,R,h); 

%% SIMULATION IN FRONT OF GIVEN CONDITIONS

tTop = 10; %maximum time for simulation
%tick = 0.0001; %granularity

x = [1; -1]; %given initial conditions

history_x = []; %buffer initialization
%history = [history sys_d.c*x];
history_x = [history_x x];
history_u = [];
history_u = [history_u 0];

for i = 0 : h : tTop
    u = -K*x; %applies control
    x = sys_d.a*x+sys_d.b*u;
    y = sys_d.c*x;
    history_x = [history_x x]; %updates buffer
    %history_y = [history y]; %updates buffer
    history_u = [history_u u];    
end

time = 0:h:tTop+h;

figure
plot(time, history_x(1,:), time, history_x(2,:), time, history_u(:));
xlabel('Time [s]');
ylabel('x, u');
legend('x[1]','x[2]','u');