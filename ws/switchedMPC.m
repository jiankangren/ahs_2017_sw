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
     
states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'u'};
outputs = {'phi'};

sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);

Q =    [  10     0   
          0    10];

R = 0.1;

%%
% Define a plant model and create a model predictive controller with MV
% constraints.
simulateMPC(0.05, sys_ss, Q, R);

simulateMPC(0.2, sys_ss, Q, R);

simulateMPC(0.1, sys_ss, Q, R);

function simulateMPC(ts,sys_ss, Q, R)
sys = c2d(sys_ss, ts);
MPCobj = mpc(sys,ts);
MPCobj.W.OutputVariables = Q;
MPCobj.W.ManipulatedVariables = R;
MPCobj.W.ManipulatedVariablesRate = 0.1;

x = mpcstate(MPCobj);
x.plant =[0;0;-0.005;0];
r = [0,0];

t = [0:ts:10];
N = length(t);
y = zeros(N,2); 
u = zeros(N,1); 
for i = 1:N
    temp = x.plant;
    u(i) = mpcmove(MPCobj,x,y(i,:),r);
    [s,temp] = ode45(@(t,temp) sys_ss.A*temp+ sys_ss.B*u(i),[0 ts],temp);
    x.plant = temp(size(temp,1),:)';
    y(i+1,:) = sys_ss.C*x.plant;
end

% plot the result.
[ts,us] = stairs(t,u);
plot(ts,us,'r-',t,y(1:length(t),1),'b--')
hold on;
plot(ts,us,'y-',t,y(1:length(t),2),'g--')
legend('MV','OV')
hold off;
end