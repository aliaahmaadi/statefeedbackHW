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


%% state space system

 A = [0 1 0 0 0
     T/W 0 0 0 H/W
     0 0 0 1 0
     G/Z 0 0 0 E/Z
     0 0 0 -(N*K_m)/L_m -R_m/L_m];
 B = [0 0 0 0 1/L_m]';
 C= eye(5,5);
 D = [0 0 0 0 0]';
 sys = ss(A,B,C,D);
 t_f  = tf(sys);
 t_f_theta =t_f (1);
 num1 = [1 14];
 den1 = [1 2];
 ct = tf(num1,den1);
 %%
 sim('nonlinear_model_advanced_control_project')
 
       
figure(1)
plot(ans.tout,ans.tn,'k','LineWidth',2)
xlabel('Time','LineWidth',2,'fontsize',14)
ylabel('\theta(radian)','LineWidth',2,'fontsize',14)
title('(Regulation \theta_0 = 35 degree)')
%legend('plant','observer')
 
figure(2)
plot(ans.tout,ans.eff,'r','LineWidth',2)
xlabel('Time','LineWidth',2,'fontsize',14)
ylabel('amplitude (voltag)','LineWidth',2,'fontsize',14)
title('actuator effort')
 %%
 
 %figure(2)
 %step(t_f_theta);
 %xlabel('Time','LineWidth',2,'fontsize',14)
 %ylabel('Amplitude of \theta','LineWidth',2,'fontsize',14)
 %title('step response for \theta (linear system)')
 %figure(3)
 %impulse(t_f_theta);
 %xlabel('Time','LineWidth',2,'fontsize',14)
 %ylabel('Amplitude of \theta','LineWidth',2,'fontsize',14)
 %title('impluse response for \theta (linear system)')
 
 %figure(4)
 %rlocus(t_f_theta)

 
 %% bass Gura method
 
% Q = ctrb(A,B);
 %syms s 
 %del = det(s*eye(5,5)-A);
 %a0 = 0; 
 %a1 = -170.9927;
 %a2 = -6494.7;
 %a3 = -74.3382;
 %a4 = 80;
 %desired poles
 %s1 = -10;
 %s2 = -13;
 %s3 = -20;
 %s4 = -15;
 %s5 = -14;
 %desired_char = expand((s+s1)*(s+s2)*(s+s3)*(s+s4)*(s+s5));

 %al0 = -546000;
 %al1 = 199300;
 %al2 = -28740;
 %al3 = 2047;
 %al4 = -72;
 
 %a = [a4 a3 a2 a1 a0]';
 %al = [al4 al3 al2 al1 al0]';
 %w = [1 a4 a3 a2 a1
  %   0 1 a4 a3 a2
   %  0 0 1 a4 a3
    % 0 0 0 1 a4
   %  0 0 0 0 1];
% k = inv((Q*w)')*(al-a);
 %k1 = k(1)
 %k2 = k(2)
 %k3 = k(3)
 %k4 = k(4)
 %k5 = k(5)
 %cl_sys = ss((A-B*k'),B,C,D);
 %step(cl_sys)
 
 %% pole placement
 %t = [-8 -1.23 -7 -3 -0.75];
 %K = place(A,B,t);

%k1 = K(1)
%k2 = K(2)
%k3 = K(3)
%k4 = K(4)
%k5 = K(5)
%A_cl = A-B*K;
%cl_sys = ss((A-B*K),B,C,D);
%step(cl_sys)
%title('step response of closed loop linear system (full state feedback)')
%t_f_cl  = tf(cl_sys);
%t_f_cl_theta =t_f_cl (1);
%rlocus(t_f_cl_theta)
%
 %title('root locus of closed loop system ')
 
 %% butherworth 
 %a_0 = 0; a_1 = 14.2858; a_2 = 179.4909; a_3 = 15.3687; a_4 = 11.0345;
 %W = [1 a_4 a_3 a_2 a_1;0 1 a_4 a_3 a_2;0 0 1 a_4 a_3;0 0 0 1 a_4; 0 0 0 0 1];
 %a = [a_0 a_1 a_2 a_3 a_4]';
 %Q = ctrb(A,B);
 %w_0 = 10;
 %z=roots([1 0 0 0 0 0 0 0 0 0 1])
 %p = w_0*[real(z(1)) real(z(2)) real(z(3)) real(z(4)) -5];
 
%al_0 = 10284403483257514762459375454373478556229011233466191960537358805/6582018229284824168619876730229402019930943462534319453394436096;
%al_1 = - (54564187372094157927492264675965978206662225545658997095627844719)/6582018229284824168619876730229402019930943462534319453394436096;
%al_2 =  (717499679589445308498719620084033637987999002147)/45671926166590716193865151022383844364247891968;
%al_3 = - (482816664062391330258061523356553)/40564819207303340847894502572032;
%al_4 = (1082167963208213)/562949953421312;
%al = [al_0 al_1 al_2 al_3 al_4]';
%K = inv((Q*W)')*(al-a);




%k1 = K(1)
%k2 = K(2)
%k3 = K(3)
%k4 = K(4)
%k5 = K(5)
 
%% full order observer 
C = [1 1 1 1 1];
%A_o = A';
%B_o = C';
%C_o = B';
%p = 2*[-8 -1.23 -7 -3 -0.75];
%L = place(A_o,B_o,p);

%l1 = L(1)
%l2 = L(2)
%l3 = L(3)
%l4 = L(4)
%l5 = L(5)
%simulation
%load_system('full_observer_2')
 %      sim('full_observer_2')
% L = [5 10 15 20 25]';
%n = eig(A - L*C)        
%figure(2)
%plot(ans.tout,ans.th_p,'b','LineWidth',2)
%hold on
%plot(ans.tout,ans.th_ob,'r','LineWidth',2)
% xlabel('Time','LineWidth',2,'fontsize',14)
%ylabel('\theta','LineWidth',2,'fontsize',14)
%title('closed loop system in presence of full state observer \theta_0 = 20 degree')
%legend('plant','observer')
 
 %figure(3)
%plot(ans.tout,ans.eff,'k','LineWidth',2)
%hold on
%plot(ans.tout,ans.phi_o,'r','LineWidth',2)
 %xlabel('Time','LineWidth',2,'fontsize',14)
 %ylabel('amplitude (voltag)','LineWidth',2,'fontsize',14)
 %title('actuator effort')
 
 
% figure(3)
%plot(ans.tout,ans.phi_p,'b','LineWidth',2)
%hold on
%5plot(ans.tout,ans.phi_ob,'r','LineWidth',2)
% xlabel('Time','LineWidth',2,'fontsize',14)
%ylabel('\phi','LineWidth',2,'fontsize',14)
% title('full state observer response ')
% legend('plant','observer')  
 
% figure(5)
%plot(ans.tout,ans.i_p,'b','LineWidth',2)
%hold on
%plot(ans.tout,ans.i_ob,'r','LineWidth',2)
%xlabel('Time','LineWidth',2,'fontsize',14)
% ylabel('current i','LineWidth',2,'fontsize',14)
%title('full state observer response i_0 = 0.75 amper')
% legend('plant','observer')
% figure(6)
%plot(ans.tout,ans.thd_p,'b','LineWidth',2)
%hold on
%plot(ans.tout,ans.thd_ob,'r','LineWidth',2)
% xlabel('Time','LineWidth',2,'fontsize',14)
% ylabel('rate of \theta (radian/second)','LineWidth',2,'fontsize',14)
%title('full state observer response rate of \theta_0 = 0.5 rad/s')
%legend('plant','observer')
%figure(7)
%plot(ans.tout,ans.phid_p,'b','LineWidth',2)
%hold on
%plot(ans.tout,ans.phid_ob,'r','LineWidth',2)
% xlabel('Time','LineWidth',2,'fontsize',14)
% ylabel('rate of \phi (radian/second)','LineWidth',2,'fontsize',14)
%title('full state observer response rate of \phi_0 = 0.5 rad/s')
%legend('plant','observer') 
