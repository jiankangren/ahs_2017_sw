ts = 0.01;
u  = [2, -1.4];
x = [2;1;3.14/2; 0];
i = 1;
while i < 500
[s,x] = ode45(@(t,x) robotModel(x,u(1),u(2)),[0 ts],x);
x = x(end,:)';
xplot(i) = x(1);
yplot(i) = x(2);
i = i +1;
end
plot(xplot,yplot);