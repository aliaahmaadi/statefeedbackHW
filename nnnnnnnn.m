clc
clear
%% artist parameters
L=1.8; % Artist's heigth
l=1; % distance between artist's hands and wire
 % rod's lenght
M_M=75; % Artist's mass
M_R=2; % rod's lenght
J_M=3.2; % artist's inertia
J_R=1.5; % rod's inertia
%% motor parameters
M_m=4;  %DC motor's %mass
M_e=0;  %encoder's mass
M_H=0;
J_H=0; % Housing's inertia
J_m=3700*10^-4; % motor's shaft inertia
j=5; % gearbox's output shaft inertia
R_m=40; % DC motor's Electric resistance
L_m=0.5; % DC motor's inductance
K_m=21; % motor's constant
N=1; % Gearbox transfer ratio
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




%% simulation
load_system('nonlinear_model_advanced_control_project')
        sim('nonlinear_model_advanced_control_project')

        
        
 figure(1)
 plot(ans.tout,ans.tn,'r')

 xlabel('Time','LineWidth',2,'fontsize',14)
 ylabel('\theta','LineWidth',2,'fontsize',14)
 title('\theta for open loop nonlinear system (\theta_0 = 5 degree)')
 
figure(2)
 plot(ans.tout,ans.phin,'k')

 xlabel('Time','LineWidth',2,'fontsize',14)
 ylabel('\phi','LineWidth',2,'fontsize',14)
 title('\phi for open loop nonlinear system')
 
 
 