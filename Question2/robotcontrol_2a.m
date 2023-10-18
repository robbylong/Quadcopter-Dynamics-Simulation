close all
clear all
syms vx vy vz px py pz sigma theta gamma wx wy wz m g u1 u2 u3 u4 k kd Ixx Iyy Izz L b
m = 0.2;
Ixx = 1;
Iyy = 1;
Izz = 0.5;
g = 9.8;
kd = 0.1;
k = 1;
L = 0.2;
b = 0.1;
Ixx = 1;
Iyy = 1;
Izz = 0.5;

R = [cos(gamma)*cos(theta) cos(gamma)*sin(sigma)*sin(theta)-cos(sigma)*sin(gamma) sin(sigma)*sin(gamma)+cos(sigma)*cos(gamma)*sin(theta);
    sin(gamma)*cos(theta) cos(gamma)*cos(sigma)+sin(theta)*sin(sigma)*sin(gamma) cos(sigma)*sin(gamma)*sin(theta)-cos(gamma)*sin(sigma);
    -sin(theta) cos(theta)*sin(sigma) cos(sigma)*cos(theta)];
TB = [0;0;k*(u1+u2+u3+u4)];
FD = [-kd*vx; -kd*vy; -kd*vz];
x2dot = [0;0;-g]+1/m*R*TB+1/m*FD;

convAngular = [1 0 -sin(theta);0 cos(sigma) cos(theta)*sin(sigma);0 -sin(sigma) cos(theta)*cos(sigma)];
inv(convAngular)
x3dot = inv(convAngular)*[wx;wy;wz];

torque = [L*k*(u1-u3); L*k*(u2-u4); b*(u1-u2+u3-u4)];
x4dot = [torque(1)/Ixx; torque(2)/Iyy; torque(3)/Izz]-[(Iyy-Izz)/Ixx*wy*wz; (Izz-Ixx)/Iyy*wx*wz; (Ixx-Iyy)/Izz*wx*wy]

f = [vx, vy, vz, x2dot(1),x2dot(2),x2dot(3),x3dot(1),x3dot(2),x3dot(3),x4dot(1),x4dot(2),x4dot(3)]
x = [px,py,pz,vx,vy,vz,sigma,theta,gamma,wx,wy,wz];
u = [u1,u2,u3,u4];
Aj = jacobian(f,x)
Bj = jacobian(f,u)
A = double(subs(Aj,[vx vy vz sigma theta gamma wx wy wz u1 u2 u3 u4],[0 0 0 0 0 0 0 0 0 0.49 0.49 0.49 0.49]))
B = double(subs(Bj,[sigma theta gamma],[0 0 0]))
% 
% syms vx vy vz px py pz sigma theta gamma wx wy wz TB torquex torquey torquez
% R = [cos(gamma)*cos(theta) cos(gamma)*sin(sigma)*sin(theta)-cos(sigma)*sin(gamma) sin(sigma)*sin(gamma)+cos(sigma)*cos(gamma)*sin(theta);
%     sin(gamma)*cos(theta) cos(gamma)*cos(sigma)+sin(theta)*sin(sigma)*sin(gamma) cos(sigma)*sin(gamma)*sin(theta)-cos(gamma)*sin(sigma);
%     -sin(theta) cos(theta)*sin(sigma) cos(sigma)*cos(theta)];
% %TB = [0;0;(u1+u2+u3+u4)];
% delta_T = TB - 49/25;
% FD = [-0.1*vx; -0.1*vy; -0.1*vz];
% x2dot = [0;0;-9.8]+1/0.2*R*[0;0;delta_T+49/25]+1/0.2*FD;
% 
% %convAngular = [1 0 -sin(theta);0 cos(sigma) cos(theta)*sin(sigma);0 -sin(sigma) cos(theta)*cos(sigma)];
% invConvAngular = [1 sin(sigma)*sin(theta)/cos(theta) cos(sigma)*sin(theta)/cos(theta);0 cos(sigma) -sin(sigma);0 sin(sigma)/cos(theta) cos(sigma)/cos(theta)];
% x3dot = invConvAngular*[wx;wy;wz];
% 
% %torque = [0.2*(u1-u3); 0.2*(u2-u4); 0.1*(u1-u2+u3-u4)];
% x4dot = [torquex; torquey; torquez*2]-[0.5*wy*wz; -0.5*wx*wz; 0];
% 
% f = [vx; vy; vz; x2dot(1);x2dot(2);x2dot(3);x3dot(1);x3dot(2);x3dot(3);x4dot(1);x4dot(2);x4dot(3)]
% x = [px;py;pz;vx;vy;vz;sigma;theta;gamma;wx;wy;wz];
% 
% u = [TB;torquex;torquey;torquez];
%x = [px;py;pz;0;0;0;0;0;0;0;0;0;];
%u = [u;u;u;u];

% % A = jacobian(f,x)
% % B = jacobian(f,u)
% A = [0 0 0 1 0 0 0 0 0 0 0 0;
%     0 0 0 0 1 0 0 0 0 0 0 0;
%     0 0 0 0 0 1 0 0 0 0 0 0;
%     0 0 0 -0.5 0 0 0 0 0 0 0 0;
%     0 0 0 0 -0.5 0 0 0 0 0 0 0;
%     0 0 0 0 0 -0.5 0 0 0 0 0 0;
%     0 0 0 0 0 0 0 0 0 1 0 0;
%     0 0 0 0 0 0 0 0 0 0 1 0;
%     0 0 0 0 0 0 0 0 0 0 0 1;
%     0 0 0 0 0 0 0 0 0 0 0 0;
%     0 0 0 0 0 0 0 0 0 0 0 0;
%     0 0 0 0 0 0 0 0 0 0 0 0;
%     ]
% B = [0 0 0 0;
%     0 0 0 0;
%     0 0 0 0;
%     0 0 0 0;
%     0 0 0 0;
%     5 5 5 5;
%     0 0 0 0;
%     0 0 0 0;
%     0 0 0 0;
%     0.2 0 -0.2 0;
%     0 0.2 0 -0.2;
%     0.2 -0.2 0.2 -0.2]
C = [1 0 0 0 0 0 0 0 0 0 0 0;
    0 1 0 0 0 0 0 0 0 0 0 0;
    0 0 1 0 0 0 0 0 0 0 0 0;
    0 0 0 1 0 0 0 0 0 0 0 0;
    0 0 0 0 1 0 0 0 0 0 0 0;
    0 0 0 0 0 1 0 0 0 0 0 0;
    0 0 0 0 0 0 1 0 0 0 0 0;
    0 0 0 0 0 0 0 1 0 0 0 0;
    0 0 0 0 0 0 0 0 1 0 0 0;
    0 0 0 0 0 0 0 0 0 1 0 0;
    0 0 0 0 0 0 0 0 0 0 1 0;
    0 0 0 0 0 0 0 0 0 0 0 1]
D = [0 0 0 0;
    0 0 0 0;
    0 0 0 0;
    0 0 0 0;
    0 0 0 0;
    0 0 0 0;
    0 0 0 0;
    0 0 0 0;
    0 0 0 0;
    0 0 0 0;
    0 0 0 0;
    0 0 0 0]
Q = [1 0 0 0 0 0 0 0 0 0 0 0;
    0 1 0 0 0 0 0 0 0 0 0 0;
    0 0 1 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 1 0 0 0 0 0;
    0 0 0 0 0 0 0 1 0 0 0 0;
    0 0 0 0 0 0 0 0 1 0 0 0;
    0 0 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 0 0;
    0 0 0 0 0 0 0 0 0 0 0 0]
R = [1 0 0 0;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1]
sys = ss(A,B,C,D);
Ts = 0.02;
sys_d = c2d(sys,Ts,'zoh')
%[K,S,e] = dlqr(sys_d.A,sys_d.B,Q,R)
save('disc_matrices.mat','sys_d')
%f-A*x-B*u