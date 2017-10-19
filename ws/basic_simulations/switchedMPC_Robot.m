cla, close all;
clear all;
l = 0.5;

C = [1 0 0 ;
     0 1 0;
     0 0 1];
D = [0 0;
     0 0
     0 0];
     
states = {'x' 'y' 'theta'};
inputs = {'v' 'phi'};
outputs = {'x' 'y', 'theta'};

R =    [  0.1   0;    
          0    0.1 ];

Q = [1000  0  0 ;
     0    1000 0;
     0    0  0];

ts = 0.01;

r = [ 2.5654;
    4.2496;
         0];
t = [0:ts:5];
N = length(t);
y = zeros(N,3); 
y_sim = zeros(N,3); 
u = zeros(N,2); 
temp =[1;1;0];
theta = temp(3);
inputV = 0;
inputPhi = 0;
PID_ref = [1,10];
kv = 0.8;
ka = 0.8;
for i = 1: N
distance = norm([PID_ref(1)-temp(1); PID_ref(2)- temp(2)]);
velInput = kv * distance;
stInput = ka * (atan2(PID_ref(2)- temp(2), PID_ref(1)-temp(1)) - temp(3));
    y_sim(i,:) = C *temp;
     [s,temp] = ode45(@(t,temp) robotModel(temp,velInput,stInput),[0 ts],temp);
     temp = temp(end,:)';
end
plot(t,y_sim(1:length(t),1),'b-')
ylabel('x');
figure;
plot(t,y_sim(1:length(t),2),'g-')
ylabel('y');
figure;
plot(y_sim(1:length(t),1),y_sim(1:length(t),2),'g-')
ylabel('y');
xlabel('x');


v = 0.0001;
phi = 0.0001;
x =[2;1;3.14/4];
theta = x(3);
%u = [1 ;3.14/2.5];
for i = 1:N
 %%   theta = x(3,i);
 %%   phi = 3.14/4;
 %%   v = 1;
    A = [0  0  -sin(theta)*v;
         0  0   cos(theta)*v;
         0  0   0];

    B = [cos(theta) 0;
         sin(theta) 0;
         tan(phi)/l v*(sec(phi)^2)/l];
 %%   sys = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);
  %%  sys_d = c2d(sys,ts);
  %%  x(:,i+1) = sys_d.A * x(:,i) + sys_d.B * u;
    sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);

    sys = c2d(sys_ss, ts);
    y(i,:) = sys.C *x;
    MPCobj = mpc(sys);
    MPCobj.W.OutputVariables = Q;
    MPCobj.W.ManipulatedVariables = R;
    MPCobj.ControlHorizon = 2;
    MPCobj.PredictionHorizon = 50;
    MPCobj.MV(1).Min = 0.001;
    MPCobj.MV(1).Max = 10;
    MPCobj.MV(1).RateMin = -5;
    MPCobj.MV(1).RateMax = 5;
    MPCobj.MV(2).RateMin = -3.14/8;
    MPCobj.MV(2).RateMax = 3.14/8;
    MPCobj.MV(2).Min = -3.14/2;
    MPCobj.MV(2).Max = 3.14/2 ;
    x_mpc = mpcstate(MPCobj);
    x_mpc.plant = x; % used for mpc
    u(i,:) = mpcmove(MPCobj,x_mpc,y(i,:),r);
    [s,x] = ode45(@(t,x) robotModel(x,u(i,1),u(i,2)),[0 ts],x);
    x = x(end,:)';
    v = u(i,1);
    phi = u(i,2);
    theta = x(3);
end
% % figure;
% % plot(t,x(1,1:length(t)),'b-')
% %  ylabel('x');
% % figure;
% % plot(t,x(2,1:length(t)),'b-')
% %  ylabel('y');
% %  
 %figure;
%plot(x(1,:),x(2,:),'b-')
%plot the result.
figure;
plot(t,y(1:length(t),1),'b-')
ylabel('x');
figure;
plot(t,y(1:length(t),2),'g-')
ylabel('y');
figure;
plot(y(1:length(t),1),y(1:length(t),2),'g-')
ylabel('y');
xlabel('x');
figure;
plot(t,u(:,1));
ylabel('velocity input');
figure;
plot(t,u(:,2));
ylabel('steering angle');