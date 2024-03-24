clc
clear 
close all
%% System parameters
L=0.1810; % Artist's heigth
l=0.0476; % distance between artist's hands and wire
 % rod's lenght
M_M=0.51; % Artist's mass
M_R=0.39; % rod's mass
M_m=0.1;  %DC motor's %mass
M_e=0.075;  %encoder's mass
M_H=0;
J_M=0.0054; % artist's inertia
J_R=0.0488; % rod's inertia
J_H=0.1; % Housing's inertia
J_m=0.032; % motor's shaft inertia
j=9.7*10^-7; % gearbox's output shaft inertia
R_m=1.6; % DC motor's Electric resistance
L_m=0.145; % DC motor's inductance
K_m=0.0109; % motor's constant
N=3; % Gearbox transfer ratio
g=9.8; % gravity acceleration
%%


J=J_R+J_H+J_M;
J_RH=J_R+J_H;
M=M_m+M_R+M_H+M_e;

W=(J_M+M*l^2)*(J_RH+N^2*J_m+j)-(J_RH)^2;
Z=(J_M+M*l^2)*(J_RH+N^2*J_m+j)^2-J_RH^2*(J_RH+N^2*J_m+j);
T=(M_M*L/2+M*l)*(J_RH+N^2*J_m+j)*g;
H=-N*K_m*J_RH;
G=-J_RH*(M_M*L/2+M*l)*(J_RH+N^2*J_m+j)*g;
E=(J_RH^2+1)*(N*K_m);

A = [0 1 0 0 0
     T/W 0 0 0 H/W
     0 0 0 1 0
     G/Z 0 0 0 E/Z
     0 0 0 -(N*K_m)/L_m -R_m/L_m];
 B = [0 0 0 0 1/L_m]';
 C = [1 1 1 1 1];
 D =0;
 g=ss(A,B,C,D);
 
 %% pole placement
 t = [-8 -1.23 -7 -3 -0.75];
 K = place(A,B,t);
 A_cl = A-B*K;
 g1 = ss(A_cl,B,C,D); 
 
 %% LQR 
 Q = [6500 0 0 0 0;0 1 0 0 0;0 0 1 0 0;0 0 0 1 0;0 0 0 0 1];
 R = 1.5;
 N = 0;
 [k_2,~,e]=lqr(g,Q,R,N);
 A_cl2 = A-B*k_2;
 g2 = ss(A_cl2,B,C,D); 
 x0 = [20*pi/180 0 0 0 0];
 t = linspace(1,20,100);
 r = 1*(t>0);
 [y,t,~]=lsim(g,r,t);%plant
 [y1,t,~]=lsim(g1,r,t);%state feedback
 [y2,t,~]=lsim(g2,r,t);%lqr
 
 
 %figure

 %plot(t,y1)
 %hold on
 %plot(t,y2)
 %legend('state feedback control','lqr conrol')
 %% 
 k1 = k_2(1);
 k2 = k_2(2);
 k3 = k_2(3);
 k4 = k_2(4);
 k5 = k_2(5);
 %% simulation
load_system('closed_loop_nonlinear_model_advanced_control_project')
       sim('closed_loop_nonlinear_model_advanced_control_project')
       
figure(1)
plot(ans.tout,ans.tcn,'b','LineWidth',1.3)
xlabel('Time','LineWidth',2,'fontsize',14)
ylabel('\theta(radian)','LineWidth',2,'fontsize',14)
title('Regulation \theta_0 = 20 degree')
grid on
 
figure(2)
plot(ans.tout,ans.effort,'k','LineWidth',1.3)
xlabel('Time','LineWidth',2,'fontsize',14)
ylabel('amplitude (voltag)','LineWidth',2,'fontsize',14)
title('actuator effort LQR controller')
grid  on
figure(3)
plot(ans.tout,ans.phii,'g','LineWidth',2)
xlabel('Time','LineWidth',2,'fontsize',14)
ylabel('\phi','LineWidth',2,'fontsize',14)
title('regulation')
grid on