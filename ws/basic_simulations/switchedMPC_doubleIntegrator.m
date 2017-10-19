cla, close all;
clear all;
ts = 0.01;
r = 5;
t = [0:ts:15];
N = length(t);
y = zeros(N,2);
y0 = zeros(N,2);
y1 = zeros(N,2);
delayFactor = 20;
maxDelayFactor = 30;
%for attackTime = 0.5/ts:50:2/ts
attackTime = 100000000000000000;
x =[0;0];
A = [ 0 1;
     0 0 ];
B = [0;1/0.5];
C = [1 0];
D = 0;
 
states = {'x' 'x_dot'};
inputs = {'u'};
outputs = {'x'};
sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);
Q = 10000;
R = 0.0001;
profile on
sys = c2d(sys_ss, ts);
MPCobj = mpc(sys);
MPCobj.W.OutputVariables = Q;
MPCobj.W.ManipulatedVariables = R;
MPCobj.ControlHorizon = 2;
MPCobj.PredictionHorizon = 100;
MPCobj.MV.Min = -5;
MPCobj.MV.Max = 5;
MPCobj.MV.RateMin = -2;
MPCobj.MV.RateMax = 2;
x_mpc = mpcstate(MPCobj);
profile viewer

sys = c2d(sys_ss, ts*maxDelayFactor);
MPCobj1 = mpc(sys);
MPCobj1.W.OutputVariables = Q;
MPCobj1.W.ManipulatedVariables = R;
MPCobj1.ControlHorizon = 2;
MPCobj1.PredictionHorizon = 100;
MPCobj1.MV.Min = -5;
MPCobj1.MV.Max = 5;
MPCobj1.MV.RateMin = -2;
MPCobj1.MV.RateMax = 2;
x_mpc1 = mpcstate(MPCobj1);
%review(MPCobj);

delay = 0;
attackCounter = 0;
delayCounter = 0;
u(1) = 0;
ux(1) = 0;
distance(1) = 0;
for i = 1:N
    if(attackCounter < attackTime)
         attackCounter = attackCounter + 1;
    else
        delay = 1;
    end
    y(i,:) = x;
    if(delay == 0)
         x_mpc.plant = x;
         u(i) = mpcmove(MPCobj,x_mpc,y(i),r);
         ux(i) = u(i) / 4;
    else 
        if(delayCounter == 0)
          x_mpc.plant = x;
          u(i) = mpcmove(MPCobj,x_mpc,y(i),r);
        else
           u(i) = u(i-1);
        end
        delayCounter = delayCounter + 1;
        delayCounter = mod(delayCounter,delayFactor);
    end
    xx = x;
    [s,x] = ode45(@(t,x) doubleIntModel(x,u(i)),[0 ts],x);
    x = x(end,:)';
    [sx,xx] = ode45(@(t,xx) doubleIntModel(xx,ux(i)),[0 ts],xx);
    xx = xx(end,:)';
    distance(i) = norm(xx(1)-x(1))
    
end
plot(distance);
x =[0;0];
delay = 0;
attackCounter = 0;
delayCounter = 0;
u0(1) = 0;
for i = 1:N
    if(attackCounter < attackTime)
         attackCounter = attackCounter + 1;
    else
        delay = 1;
    end
    y0(i,:) = x;
    if(delay == 0)
         x_mpc.plant = x;
         u0(i) = mpcmove(MPCobj,x_mpc,y0(i),r);
    else 
        if(delayCounter == 0)
          x_mpc1.plant = x;
          u0(i) = mpcmove(MPCobj1,x_mpc1,y0(i),r);
        else
           u0(i) = u0(i-1);
        end
        delayCounter = delayCounter + 1;
        delayCounter = mod(delayCounter,delayFactor);
    end
    [s,x] = ode45(@(t,x) doubleIntModel(x,u0(i)),[0 ts],x);
    x = x(end,:)';
end

% figure;
% plot(t,y(1:length(t),1),'b-')
% figure;
% plot(t,u,'g-');

x =[0;0];
delay = 0;
attackCounter = 0;
delayCounter = 0;
u1(1) = 0;
for i = 1:N
    if(attackCounter < attackTime)
         attackCounter = attackCounter + 1;
    else
        delay = 1;
    end
    y1(i,:) = x;
    if(delay == 0)
         x_mpc.plant = x;
         u1(i) = mpcmove(MPCobj,x_mpc,y1(i),r);
    else 
        if(delayCounter == 0)
            x_mpc.plant = x;
            x_mpc1.plant = x;
            input1 = mpcmove(MPCobj,x_mpc,y1(i),r);
            input2 = mpcmove(MPCobj1,x_mpc1,y1(i),r);
            u1(i) = (input1 + input2)* (delayFactor*ts)/((maxDelayFactor*ts)+ts);
        else
           u1(i) = u1(i-1);
        end
        delayCounter = delayCounter + 1;
        delayCounter = mod(delayCounter,delayFactor);
    end
    [s,x] = ode45(@(t,x) doubleIntModel(x,u1(i)),[0 ts],x);
    x = x(end,:)';
end

% plot the result.
figure('Name', num2str(attackTime*ts));
plot(t,y(1:length(t),1),'r-')
hold on;
plot(t,y1(1:length(t),1),'g-')
plot(t,y0(1:length(t),1),'b-')
hold off;

figure('Name', num2str(attackTime*ts));
plot(t,u1,'g-')
hold on;
ylim([-6 6]);
plot(t,u,'r-')
plot(t,u0,'b');
%end
