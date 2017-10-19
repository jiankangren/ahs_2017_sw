 cla, close all
M = 0.5;
m = 0.2;
b = 0.1;
I = 0.006;
g = 9.8;
l = 0.3;

p = I*(M+m)+M*m*l^2; %denominator for the A and B matrices

A = [0      1              0           0;
     0 -(I+m*l^2)*b/p  (m^2*g*l^2)/p   0;
     0      0              0           1;
     0 -(m*l*b)/p       m*g*l*(M+m)/p  0];
B = [     0;
     (I+m*l^2)/p;
          0;
        m*l/p];
C = [1 0 0 0;
     0 0 1 0];
D = [0;
     0];
 
Apend = [  0     1     0     0 ;
     0     0    -1     0 ;
     0     0     0     1 ;
     0     0     9     0 ];
Bpend = [
         0 ;
    0.1000 ;
         0 ;
   -0.1000] ;
Cpend = [0     0     1     0 ]
Dpend =0 ;
     
states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'u'};
outputs = {'phi'};

sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);
% bode(sys_ss)

Q =    [  1     0     0     0 ;
     0     1     0     0 ;
     0     0    10     0 ;
     0     0     0    10 ];

R = 0.1;
T = 0.05;
K = lqr(A,B,Q,R);
Acc = [(A-B*K)];

sys_cl_c = ss(Acc,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);
% figure
% bode(sys_cl_c)

% convert to discrete model
% calculate LQR gains for T = 0.05
sys_1 = c2d(sys_ss,T);
[K_1,P] = dlqr(sys_1.A,sys_1.B,Q*T,R/T);

T2 = 0.2;
sys_2 = c2d(sys_ss, T2);
[K_2,P2] = dlqr(sys_2.A,sys_2.B,Q*T2,R/T2);

T3 = 0.24;
sys_3 = c2d(sys_ss, T2);
[K_3,P3] = dlqr(sys_3.A,sys_3.B,Q*T3,R/T3);

% Ac = [(A-B*K_1)];
% sys_cl_d = ss(Ac,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);
% % figure
% % bode(sys_cl_d)
% 
figure
t = 0:T:20;
r = 0.2*ones(size(t));
[y,x] = lsim(sys_cl_c,r,t);
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','cart position (m)')
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)')
title('Step Response with continuous LQR Control')
% 
% figure
% r = 0.2*ones(size(t));
% [y,x] = dlsim(Ac,B,C,D,r);
% [AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
% set(get(AX(1),'Ylabel'),'String','cart position (m)')
% set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)')
% title('Step Response with discrete LQR Control')



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% figure
% x = 0;
% y = 0;
% x(1:4,1) = [0;0;-2;0];
% y(1:2,1) = [0;0];
% 
% k = 1;
% for t =0:T:20
%     x(:,k+1) = (sys_1.A - sys_1.B*K_1) *x(:,k);
%     y(:,k+1) = sys_1.C*x(:,k+1);
%     k = k + 1;
% end
% time = 0:T:20;
% plot(time,x(1,1:size(time,2)));
% hold on;
% plot(time,x(3,1:size(time,2)));
%%%%%%%%%%%%%%%%%%%%%%%%%
% normal operation
figure
x = 0;
y = 0;
x(1:4,1) = [0;0;-0.005;0];
y(1:2,1) = [0;0];
k = 1;
for t =0:T:20
    u = - K_1 *x(:,k);
    m = x(:,k);
    [s,m] = ode45(@(t,m) sys_ss.A*m + sys_ss.B*u,[0 T],m);
    x(:,k+1) = m(size(m,1),:)';
    y(:,k+1) = sys_1.C*x(:,k+1);
    k = k + 1;
end
time = 0:T:20;
title('LQR Controller with ts = 0.05')
yyaxis left
plot(time,x(1,1:size(time,2)));
ylabel('cart position (m)');
hold on;
yyaxis right
ylabel('pendulum angle (radians)');
plot(time,x(3,1:size(time,2)));

%%%%%%%%%%%%%%%%%%%%%%
% %controller delay
figure
x = 0;
y = 0;
x(1:4,1) = [0;0;-0.005;0];
y(1:2,1) = [0;0];

k = 1;
controllerDelay = 0;
for t =0:T:20
    m = x(:,k);
    if(controllerDelay == 0)
        m1 = m;
    end
    if (controllerDelay < 5)
        u = - K_1 *m1;
        controllerDelay = controllerDelay + 1;
    else
        u = - K_1 *m;
        controllerDelay = 0;
    end
    [s,m] = ode45(@(t,m) sys_ss.A*m + sys_ss.B*u,[0 T],m);
    x(:,k+1) = m(size(m,1),:)';
    y(:,k+1) = sys_1.C*x(:,k+1);
    k = k + 1;
end
time = 0:T:20;
title('Slow controller with tc = 0.25')
yyaxis left
plot(time,x(1,1:size(time,2)));
ylabel('cart position (m)');
hold on;
yyaxis right
ylabel('pendulum angle (radians)');
plot(time,x(3,1:size(time,2)));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%measurements delay without switching
figure
x = 0;
y = 0;
time = 0;
x(1:4,1) = [0;0;-0.005;0];
y(1:2,1) = [0;0];
T = 0.05;
k = 1;
gain = K_1;
sP1 = 2.5;
sP2 = 7.5;
for t =0:T:sP1
    time(k) = t;
    u = - gain *x(:,k);
    m = x(:,k);
    [s,m] = ode45(@(t,m) sys_ss.A*m + sys_ss.B*u,[0 T],m);
    x(:,k+1) = m(size(m,1),:)';
    y(:,k+1) = sys_1.C*x(:,k+1);
    k = k + 1;
end
for t =sP1+T2:T2:sP2
    time(k) = t;
    u = - gain *x(:,k);
    m = x(:,k);
    [s,m] = ode45(@(t,m) sys_ss.A*m + sys_ss.B*u,[0 T2],m);
    x(:,k+1) = m(size(m,1),:)';
    y(:,k+1) = sys_1.C*x(:,k+1);
    k = k + 1;
end
for t =sP2+T2:T2:20
    time(k) = t;
    u = - gain *x(:,k);
    m = x(:,k);
    [s,m] = ode45(@(t,m) sys_ss.A*m + sys_ss.B*u,[0 T3],m);
    x(:,k+1) = m(size(m,1),:)';
    y(:,k+1) = sys_1.C*x(:,k+1);
    k = k + 1;
end
%time = 0:T:20;
title('Changing sensor rates without switching controller')
yyaxis left
plot(time,x(1,1:size(time,2)));
ylabel('cart position (m)');
figure
hold on;
yyaxis right
ylabel('pendulum angle (radians)');
plot(time,x(3,1:size(time,2)));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%measurements delay with switching
figure
x = 0;
y = 0;
time = 0;
x(1:4,1) = [0;0;-0.005;0];
y(1:2,1) = [0;0];
T = 0.05;
k = 1;
gain = K_1;
sP1 = 2.5;
sP2 = 7.5;
for t =0:T:sP1
    time(k) = t;
    u = - gain *x(:,k);
    m = x(:,k);
    [s,m] = ode45(@(t,m) sys_ss.A*m + sys_ss.B*u,[0 T],m);
    x(:,k+1) = m(size(m,1),:)';
    y(:,k+1) = sys_1.C*x(:,k+1);
    k = k + 1;
end
gain = K_2;
for t =sP1+T2:T2:sP2
    time(k) = t;
    u = - gain *x(:,k);
    m = x(:,k);
    [s,m] = ode45(@(t,m) sys_ss.A*m + sys_ss.B*u,[0 T2],m);
    x(:,k+1) = m(size(m,1),:)';
    y(:,k+1) = sys_1.C*x(:,k+1);
    k = k + 1;
end
gain = K_3;
for t =sP2+T2:T2:20
    time(k) = t;
    u = - gain *x(:,k);
    m = x(:,k);
    [s,m] = ode45(@(t,m) sys_ss.A*m + sys_ss.B*u,[0 T3],m);
    x(:,k+1) = m(size(m,1),:)';
    y(:,k+1) = sys_1.C*x(:,k+1);
    k = k + 1;
end

%time = 0:T:20;
title('Changing sensor rates, switching LQR Controller accordingly')
yyaxis left
plot(time,x(1,1:size(time,2)));
ylabel('cart position (m)');
hold on;
yyaxis right
ylabel('pendulum angle (radians)');
plot(time,x(3,1:size(time,2)));