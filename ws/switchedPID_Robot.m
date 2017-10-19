cla, close all;
l = 0.5;

C = [1 0 0 ;
     0 1 0
     0 0 1];
D = [0 0;
     0 0;
     0 0];
     
states = {'x' 'y' 'theta'};
inputs = {'v' 'phi'};
outputs = {'x' 'y' 'theta'};


R =    [  1   0;    
          0    1 ];

Q = [10   0   0;
     0    10   0;
     0    0    0];

ts = 0.05;
r = [2,1,0];
t = [0:ts:10];
N = length(t);
y = zeros(N,3); 
u = zeros(N,2); 
v = 0.01;
phi = 0;
x =[1;1;0];
%theta = temp(3);
% for i = 1: N
%      [s,temp] = ode45(@(t,temp) robotModel(temp,1,0.5),[0 ts],temp);
%      temp = temp(size(temp,1),:)';
%      y(i+1,:) = temp;
% end
% % plot the result.
% [ts,us] = stairs(t,u);
% plot(t,y(1:length(t),1),'b--')
% %hold on;
% figure;
% plot(t,y(1:length(t),2),'g--')
% figure;
% plot(y(1:length(t),1), y(1:length(t),2));
% %hold off;
kpp = 5;
kpi = 0.00;
positionIntegral = 0;
kap = 5;
kai = 0.00;
angleIntegral = 0;
for i = 1:N
    %     A = [0  0  -sin(theta)*cos(phi)*v;
%                 0  0   cos(theta)*cos(phi)*v;
%                 0  0   0];
% 
%     B = [cos(theta)*cos(phi) -sin(phi)*cos(theta)*v;
%                 sin(theta)*cos(phi) -sin(phi)*sin(theta)*v;
%                 sin(phi)/l v*cos(phi)/l];
%     sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);
% 
%     sys = c2d(sys_ss, ts);
%     MPCobj = mpc(sys);
%     MPCobj.W.OutputVariables = Q;
%     MPCobj.W.ManipulatedVariables = R;
%     MPCobj.W.ManipulatedVariablesRate = [1 1];
%    % MPCobj.Model.Plant=minreal(MPCobj.Model.Plant)
%     x = mpcstate(MPCobj);
%     x.plant = temp; % used for ode
%     y(i,:) = (sys_ss.C*x.plant)';
%     u(i,:) = mpcmove(MPCobj,x,y(i,:),r);
    positionError = norm([r(1)- x(1)  r(2) - x(2)]);
    angleError = x(3) - atan2(r(1) - x(1), r(2) - x(2));
    positionIntegral = positionIntegral + ts * positionError;
    angleIntegral = angleIntegral + ts * angleError;
    v = kpp* (positionError) + kpi * (positionIntegral);
    phi = kap * (angleError) + kai * (angleIntegral);
    [s,x] = ode45(@(t,x) robotModel(x,v,phi),[0 ts],x);
    x = x(size(x,1),:)'
    y(i,:) = x;
end

% plot the result.
[ts,us] = stairs(t,u(:,1));
[ts2,us2] = stairs(t,u(:,2));
plot(t,y(1:length(t),1),'b-')
figure;
plot(t,y(1:length(t),2),'g-')
figure;
plot(y(1:length(t),1),y(1:length(t),2),'g-')
