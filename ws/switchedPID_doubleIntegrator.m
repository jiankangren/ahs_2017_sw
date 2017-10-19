cla, close all;
clear all;
ts = 0.01;
r = 5;
t = [0:ts:100];
N = length(t);
y = zeros(N,2); 
delayFactor = 10;

x =[0;0];
kpp =1;
kdi = 1.5;
kpi = 0.07;
% kpp = 0.1;
% kdi = 1.0;
% kpi = 0.0005;

positionIntegral = 0;
previousError = 0;
noDelay = 0;
delayCounter = 0;
u(1) = 0;
for i = 1:N
    positionError = r- x(1);
    positionIntegral = positionIntegral + ts * positionError;
    positionDerivative = (positionError - previousError)/ts;
    y(i,:) = x;
    if(noDelay)
         u(i)= kpp* (positionError) + kpi * (positionIntegral) + kdi * (positionDerivative);
    else
        if(delayCounter == 0)
            u(i)= kpp* (positionError) + kpi * (positionIntegral) + kdi * (positionDerivative);
            u(i) = u(i) / 2 ;%delayFactor;
        else
          u(i) = u(i-1);
        end
    end
    if(u(i) > 20)
        u(i) = 20;
    elseif (u(i) < -20)
        u(i) = -20;
    end
   % u(i) = u(i)/2;
    [s,x] = ode45(@(t,x) doubleIntModel(x,u(i)),[0 ts],x);
    x = x(size(x,1),:)';
    previousError = positionError;
    delayCounter = delayCounter + 1;
    delayCounter = mod(delayCounter,delayFactor);
end

% plot the result.
[ts,us] = stairs(t,u);
[ts2,us2] = stairs(t,u);
plot(t,y(1:length(t),1),'b-')
figure;
plot(t,u,'g-')
% figure;
% plot(t,y(1:length(t),2),'g-')
% figure;
% plot(t,u,'g-')
