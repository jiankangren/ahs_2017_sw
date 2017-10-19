close all;
clearvars;
rosshutdown();
rosinit();
filePath = fullfile(fileparts(which('PathPlanningExample')),'data','exampleMaps.mat');
load(filePath);
map = robotics.BinaryOccupancyGrid(simpleMap, 2);
robotRadius = 0.2;
mapInflated = copy(map);
inflate(mapInflated,robotRadius);
show(mapInflated);
hold on;
prm = robotics.PRM;
prm.Map = mapInflated;
prm.NumNodes = 50;
prm.ConnectionDistance = 1;
startLocation = [2 4];
endLocation = [4 8];
path = [%2.1 4.2 0.05;
    %   2.2       4.5    0.3;
   % 2.83 7 0.1;
    2.2    4.4 0.2;
    3    5.2 0.4;
    % 2.9     6.3  0.3;
  %  3.2       7.2  0.3;
    3.2       6  0.3;
    3.2    7.35  0.3;
   % 2.75   7.75  0.3
    2.4       8    0.3;
    2       8.5 0.3;
   % 2       9 0.3;
];
path = flip(path);
l =1;
% path =  findpath(prm, startLocation, endLocation)
%safe_x = [3 3 4.5   4.5  12.5 12.5 7.5  7.5 2.5 2.5 1   1]';
%safe_y = [0 2.5 2.5 7     7    12 12 9   9   5.5 5.5 0]';
%more safe boundaries
%plot(path(:,1), path(:,2), 'x');
safe_x = [2.75 2.75 4.25   4.25 7.75 7.75 6.25 6.25 12.25 12.25 9.25 9.25 12.25 12.25 7.75  7.75  3.75 3.75  5.75  5.75  1.25  1.25 2.75 2.75 1.25    1.25 2.75]';
safe_y = [0.25 2.75 2.75   7.25 7.25 5.25 5.25 1.25 1.25   2.25 2.25 7.25 7.25  11.75 11.75 8.75  8.75 10.75 10.75 11.75 11.75 7.25 7.25 5.25 5.25 0.25 0.25]';
%plot(safe_x,safe_y);
x_X =[2;0];
x_Y =[1;0];

ts = 0.1;
%t = [0:ts:15];
%N = length(t);
y_X = zeros(1,2);
y_Y = zeros(1,2);



maxDelayFactor =26;%2.25
delay = 0;
adaptation = 1;
conservative = 1;



delayFactor = maxDelayFactor;

N = 5;
H = 1;
attackTime = 1;
A = [ 0 1;
     0 0 ];
B = [0;1/0.5];
C = [1 0];
D = 0;
alpha = 0.7;
C = [1 0 0 0;
     0 1 0 0;
     0 0 1 0;
     0 0 0 1];
D = [0 0;
     0 0;
     0 0;
     0 0];
     
states_sys = {'x' 'y' 'theta','gamma'};
inputs = {'v' 'phi'};
outputs = {'x' 'y', 'theta', 'gamma'};

R =    [  1  0;    
         0   1];

Q = [1000  0  0  0;
     0    1000 0 0;
     0    0     1 0;
     0    0     0 0];
 

phi = 0.0001;
x =[2;1;3.14/4;0];
theta = x(3);


%review(MPCobj);

attackCounter = 0;
delayCounter = 0;
estimatedDelayFactor = 1;

u = [0.0001,0];
time = 0;
index = 1;
states_sub = rossubscriber('/mpc_vel','geometry_msgs/Twist');
for goalIndex = 1 : length(path)    
    r_X = path(goalIndex,1);
    r_Y = path(goalIndex,2);
    r = [r_X; r_Y; 3.14/4;0];
    epsilon = path(goalIndex,3);
    distance = norm([r_X - x(1) r_Y - x(1)]);
    while (distance > epsilon)
        states = receive(states_sub,10);
        
        x(1) = states.Linear.X +2.3829;
        x(2) = states.Linear.Y + 6.7;
        x(3) = states.Angular.Z;
        x(4) = u(2);
        v = u(1);
        phi = u(2);
        theta = x(3);
        
        A = [0  0  -sin(theta)*v 0;
         0  0   cos(theta)*v 0;
         0  0   0  v*(sec(x(4))^2)/l;
         0  0   0   0];

        B = [cos(theta) 0;
             sin(theta) 0;
             tan(x(4))/l 0;
             0           1];
        % continuous state space
        sys_ss = ss(A,B,C,D,'statename',states_sys,'inputname',inputs,'outputname',outputs);
        y = C *x;
 
         if(delay == 0)
            sys = c2d(sys_ss, ts);
            MPCobj = mpc(sys);
            MPCobj.W.OutputVariables = Q;
            MPCobj.W.ManipulatedVariables = R;
            MPCobj.ControlHorizon = 10;
            MPCobj.PredictionHorizon = 100;
            MPCobj.MV(1).Min =  0.01;
            MPCobj.MV(1).Max = 2;
            MPCobj.MV(1).RateMin = -0.2;
            MPCobj.MV(1).RateMax = 0.2;
            MPCobj.MV(2).Min = -0.2;
            MPCobj.MV(2).Max = 0.2 ;
            x_mpc = mpcstate(MPCobj);
            x_mpc.plant = x; % used for mpc
            u = mpcmove(MPCobj,x_mpc,y,r);
        else 
            if(delayCounter == 0)
                if(adaptation == 0)
                    sys = c2d(sys_ss, ts);
                    MPCobj = mpc(sys);
                    MPCobj.W.OutputVariables = Q;
                    MPCobj.W.ManipulatedVariables = R;
                    MPCobj.ControlHorizon = 10;
                    MPCobj.PredictionHorizon = 100;
                    MPCobj.MV(1).Min =  0.0;
                    MPCobj.MV(1).Max = 2;
                    MPCobj.MV(1).RateMin = -2;
                    MPCobj.MV(1).RateMax = 2;
                    MPCobj.MV(2).Min = -1.4;
                    MPCobj.MV(2).Max = 1.4 ;
                    x_mpc = mpcstate(MPCobj);
                    x_mpc.plant = x; % used for mpc
                    u = mpcmove(MPCobj,x_mpc,y,r);
                else
                    if(conservative == 1)
                        sys = c2d(sys_ss, ts*maxDelayFactor);
                        MPCobj = mpc(sys);
                        MPCobj.W.OutputVariables = Q;
                        MPCobj.W.ManipulatedVariables = R;
                        MPCobj.ControlHorizon = 10;
                        MPCobj.PredictionHorizon = 100;
                        MPCobj.MV(1).Min =  0.0;
                        MPCobj.MV(1).Max = 2;
                        MPCobj.MV(1).RateMin = -2;
                        MPCobj.MV(1).RateMax = 2;
                        MPCobj.MV(2).Min = -1.4;
                        MPCobj.MV(2).Max = 1.4 ;
                        x_mpc = mpcstate(MPCobj);
                        x_mpc.plant = x; % used for mpc
                        u = mpcmove(MPCobj,x_mpc,y,r);
                    else
                        %estimate delay TODO: add EWMA
                        estimatedDelayFactor = (1-alpha)*estimatedDelayFactor + alpha *delayFactor;
                        delcomposedayPlot(index) = (delayFactor-1)*100;
                        estimatedDelayPlot(index) = (estimatedDelayFactor-1)*100;
                        %Adaptation
                       
                        sys = c2d(sys_ss, ts*maxDelayFactor);
                        MPCobj = mpc(sys);
                        MPCobj.W.OutputVariables = Q;
                        MPCobj.W.ManipulatedVariables = R;
                        MPCobj.ControlHorizon = 10;
                        MPCobj.PredictionHorizon = 100;
                        MPCobj.MV(1).Min =  0.0;
                        MPCobj.MV(1).Max = 2;
                        MPCobj.MV(1).RateMin = -2;
                        MPCobj.MV(1).RateMax = 2;
                        MPCobj.MV(2).Min = -1.4;
                        MPCobj.MV(2).Max = 1.4 ;
                        x_mpc = mpcstate(MPCobj);
                        x_mpc.plant = x; % used for mpc
                        u_max = mpcmove(MPCobj,x_mpc,y,r);

                     
                        sys = c2d(sys_ss, ts&estimateDelayFactor);
                        MPCobj = mpc(sys);
                        MPCobj.W.OutputVariables = Q;
                        MPCobj.W.ManipulatedVariables = R;
                        MPCobj.ControlHorizon = 10;
                        MPCobj.PredictionHorizon = 100;
                        MPCobj.MV(1).Min =  0.0;
                        MPCobj.MV(1).Max = 2;
                        MPCobj.MV(1).RateMin = -2;
                        MPCobj.MV(1).RateMax = 2;
                        MPCobj.MV(2).Min = -1.4;
                        MPCobj.MV(2).Max = 1.4 ;
                        x_mpc = mpcstate(MPCobj);
                        x_mpc.plant = x; % used for mpc
                        u_e = mpcmove(MPCobj,x_mpc,y,r);
                        
                        risk = sqrt(((estimatedDelayFactor - delayFactor)/maxDelayFactor)^2);
                        u = u_max + (1 - risk) * (u_e-u_max);
                    end
                end
                %  delayFactor = maxDelayFactor;
                %  delayFactor =  randi([1 maxDelayFactor],1,1);
                if(mod(index,124) == 0)
                  if (delayFactor == 1)
                      delayFactor = maxDelayFactor;          
                  else
                      delayFactor = 1;
                  end
                end
            end
            delayCounter = delayCounter + 1;
            delayCounter = mod(delayCounter,delayFactor);
         end
       
            
        chatpub = rospublisher('/cmd_acc','geometry_msgs/Twist');
        msg = rosmessage(chatpub);
        msg.Linear.X = u(1);
        msg.Angular.Z = u(2);
      %   if(index <5)
      %       msg.Linear.X = 0;
      %       msg.Angular.Z = -0.045;
      %   end

        send(chatpub,msg);
        index = index + 1;
     %   [s,x_X] = ode45(@(t,x_X) doubleIntModel(x_X,u_X),[0 ts],x_X);
      %  x_X = x_X(end,:)';
      %  [s,x_Y] = ode45(@(t,x_Y) doubleIntModel(x_Y,u_Y),[0 ts],x_Y);
      %  x_Y = x_Y(end,:)';
        distance = norm([r_X - y_X(1) r_Y - y_Y(1)]);
      % ROS
       %circle = viscircles([y_X(1) y_Y(1)], 0.25);
       %circle.centers =[y_X(1) y_Y(1)];
        plot(x(1), x(2), 'b--o');
        time = time + ts;
    end
end
stobpub = rospublisher('/stop','std_msgs/Bool');
msg = rosmessage(stobpub);
msg.Data = true;
send(stobpub,msg);
time
title('');
%title(strcat('Random Delays with adaptation algorithm. Time = ', num2str(time)));
hold off;
figure
hold on
plot(estimatedDelayPlot);
xlabel('Controller time steps');
ylabel('Controller Overhead %');
plot(delayPlot);
legend('Estimated Overhead', 'Actual Overhead');