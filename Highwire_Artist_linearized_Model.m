clc
clear

A=[0 1 0 0 0
    20.3005 0 0 0 -0.7060
    0 0 0 1 0
    -8.8088 0 0 0 1.4178
    0 0 0 -12 -14]
B=[0 0 0 0 20]'
C=eye(5)
D=zeros(5,1)
t=0:0.1:10;
u=sin(t);
figure(1)
lsim(A,B,C,0,u,t)
%%
[num denum]=ss2tf(A,B,C,D);
printsys(num,denum)
%%
poles=eig(A)
co=ctrb(A,B)
ob=obsv(A,C)
controlability=rank(co)
obsevability=rank(ob)
k=place(A,B,[-1 -1.5 -1.7 -2 -3])


figure(2)
lsim(A-B*k,B,C,0,u,t)
