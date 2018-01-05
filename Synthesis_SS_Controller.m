% File: Synthesis_SS_Controller.m
% Description: Synthesis of SS Controller in Matlab using Simulink
% Environment: Matlab
%
% MIT License
% Copyright (c) 2017 Valentyn N Sichkar
% github.com/sichkar-valentyn
% Reference to:
% [1] Valentyn N Sichkar. Synthesis of SS Controller in Matlab using Simulink // GitHub platform [Electronic resource]. URL: https://github.com/sichkar-valentyn/Synthesis_SS_Controller (date of access: XX.XX.XXXX)

close all;
% Plant simulation
Kpc = 3; Tpc = 0.001;
Ra = 10; Ta = 0.01; Ke = 0.05; Km = Ke;
J = 0.5E-5; Kvs = 10/600;
Ax = [   0      Km/J         0 ;
    -Ke/(Ra*Ta) -1/Ta     1/(Ra*Ta) ;
         0       0         -1/Tpc ];
Bu = [0; 0; Kpc/Tpc];
Cx = [Kvs 0 0]; Du = 0;
Plant_ss = ss(Ax, Bu, Cx, Du);
Plant_tf = tf(Plant_ss);
Tend = 0.12;
[y, t] = step(Plant_ss, Tend);
figure(1); plot(t, y, '--k'); grid on; hold on;
           axis([0 Tend -0.1 1.2]);
           legend('y'); xlabel('t, c');
 
% Controller synthesis with place function
% P = [-100; -50+100i; -50-100i]; % 1 var
% P = [-200; -100+100i; -100-100i]; % 2 var
P = [-50; -50+50i; -50-50i]; % 3 var

KK = place(Ax, Bu, P);
Sys_closed = ss(Ax - Bu*KK, Bu, Cx, Du);
Ksys = dcgain(Sys_closed);
Kg = 1/Ksys;

% Simulation of closed-loop system 
[y_c, t] = step(Sys_closed, Tend); 
figure(2); plot(t, Kg*y_c, 'k');
legend('y', 'K_g*y_c');

% Simulation of the system using Simulink model 
System_without_Observer;
Model = 'System_without_Observer';
g = 1;
X0 = [0; 0; 0];
C2 = eye(3,3);
D2 = [0; 0; 0];
[t, x, Yc, U] = sim(Model, Tend);
figure(3); 
   subplot(211);
       plot(t, Yc, 'k'); grid on; legend('y_C_L');
       axis([0 Tend -0.1 1.2]); xlabel('t, c');
   subplot(212); plot(t, U, 'k'); grid on; legend('u'); 
       axis([0 Tend -0.1 2.0]); xlabel('t, c');
