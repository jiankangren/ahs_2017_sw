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
delay = 1;
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
states = {'x' 'x_dot'};
inputs = {'u'};
outputs = {'x'};
sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);
Q = 1000;
R = 10;

sys = c2d(sys_ss, ts);
MPCobj_X = mpc(sys);
MPCobj_X.W.OutputVariables = Q;
MPCobj_X.W.ManipulatedVariables = R;
MPCobj_X.ControlHorizon = 2;
MPCobj_X.PredictionHorizon = 100;
MPCobj_X.MV.Min = -0.07;
MPCobj_X.MV.Max = 0.07;
MPCobj_X.MV.RateMin = -0.045;
MPCobj_X.MV.RateMax = 0.045;
state_mpc_X = mpcstate(MPCobj_X);

MPCobjY = mpc(sys);
MPCobjY.W.OutputVariables = Q;
MPCobjY.W.ManipulatedVariables = R;
MPCobjY.ControlHorizon = 2;
MPCobjY.PredictionHorizon = 100;
MPCobjY.MV.Min = -0.07;
MPCobjY.MV.Max = 0.07;
MPCobjY.MV.RateMin = -0.045;
MPCobjY.MV.RateMax = 0.045;
state_mpc_Y = mpcstate(MPCobjY);

%review(MPCobj);

attackCounter = 0;
delayCounter = 0;
estimatedDelayFactor = 1;
%u_X(1) = 0;
%u_Y(1) = 0;
u_X = 0;
u_Y = 0;
time = 0;
index = 1;
states_sub = rossubscriber('/mpc_vel','geometry_msgs/Twist');
for goalIndex = 1 : length(path)    
    r_X = path(goalIndex,1);
    r_Y = path(goalIndex,2);
    epsilon = path(goalIndex,3);
    distance = norm([r_X - y_X(1) r_Y - y_Y(1)]);
    while (distance > epsilon)
        states = receive(states_sub,10);
        x_X(1) = states.Linear.X +2.3829;
        x_X(2) = states.Angular.X;
        x_Y(1) = states.Linear.Y + 6.7;
        x_Y(2) = states.Angular.Y;
        y_X = x_X;
        y_Y = x_Y;
         if(delay == 0)
             state_mpc_X.plant = x_X;
             state_mpc_Y.plant = x_Y;
             u_X = mpcmove(MPCobj_X,state_mpc_X,y_X(1),r_X)
             u_Y = mpcmove(MPCobjY,state_mpc_Y, y_Y(1),r_Y)
        else 
            if(delayCounter == 0)
                if(adaptation == 0)
                     state_mpc_X.plant = x_X;
                     state_mpc_Y.plant = x_Y;
                     u_X = mpcmove(MPCobj_X,state_mpc_X,y_X(1),r_X);
                     u_Y = mpcmove(MPCobjY,state_mpc_Y, y_Y(1),r_Y);
                else
                    if(conservative == 1)
                        sys = c2d(sys_ss, ts*maxDelayFactor);
                        MPCobj_X = mpc(sys);
                        MPCobj_X.W.OutputVariables = Q;
                        MPCobj_X.W.ManipulatedVariables = R;
                        MPCobj_X.ControlHorizon = 2;
                        MPCobj_X.PredictionHorizon = 100;
                        MPCobj_X.MV.Min = -0.07;
                        MPCobj_X.MV.Max = 0.07;
                        MPCobj_X.MV.RateMin = -0.045;
                        MPCobj_X.MV.RateMax = 0.045;
                        state_mpc_X = mpcstate(MPCobj_X);

                        MPCobjY = mpc(sys);
                        MPCobjY.W.OutputVariables = Q;
                        MPCobjY.W.ManipulatedVariables = R;
                        MPCobjY.ControlHorizon = 2;
                        MPCobjY.PredictionHorizon = 100;
                        MPCobjY.MV.Min = -0.07;
                        MPCobjY.MV.Max = 0.07;
                        MPCobjY.MV.RateMin = -0.045;
                        MPCobjY.MV.RateMax = 0.045;
                        state_mpc_Y = mpcstate(MPCobjY);
                        state_mpc_X.plant = x_X;
                        state_mpc_Y.plant = x_Y;
                        u_X = mpcmove(MPCobj_X,state_mpc_X,y_X(1),r_X);
                        u_Y = mpcmove(MPCobjY,state_mpc_Y, y_Y(1),r_Y);
                    else
                        %estimate delay TODO: add EWMA
                        estimatedDelayFactor = (1-alpha)*estimatedDelayFactor + alpha *delayFactor;
                        delcomposedayPlot(index) = (delayFactor-1)*100;
                        estimatedDelayPlot(index) = (estimatedDelayFactor-1)*100;
                        %Adaptation
                       
                        sys_e = c2d(sys_ss, ts*estimatedDelayFactor);
                        MPCobjXe = mpc(sys_e);
                        MPCobjXe.W.OutputVariables = Q;
                        MPCobjXe.W.ManipulatedVariables = R;
                        MPCobjXe.ControlHorizon = 2;
                        MPCobjXe.PredictionHorizon = 50;
                        MPCobjXe.MV.Min = -5;
                        MPCobjXe.MV.Max = 5;
                        MPCobjXe.MV.RateMin = -2;
                        MPCobjXe.MV.RateMax = 2;
                        state_mpc_Xe = mpcstate(MPCobjXe);

                        MPCobjYe = mpc(sys_e);
                        MPCobjYe.W.OutputVariables = Q;
                        MPCobjYe.W.ManipulatedVariables = R;
                        MPCobjYe.ControlHorizon = 2;
                        MPCobjYe.PredictionHorizon = 50;
                        MPCobjYe.MV.Min = -5;
                        MPCobjYe.MV.Max = 5;
                        MPCobjYe.MV.RateMin = -2;
                        MPCobjYe.MV.RateMax = 2;
                        state_mpc_Ye = mpcstate(MPCobjYe);
                        state_mpc_Xe.plant = x_X;
                        state_mpc_Ye.plant = x_Y;
                        % control input based on estimated delay
                        [u_Xe, info_Xe] = mpcmove(MPCobjXe,state_mpc_Xe,y_X(1),r_X);
                        [u_Ye, info_Ye] = mpcmove(MPCobjYe,state_mpc_Ye,y_Y(1),r_Y);
                       
                        % Calculate Umax
                        sys_max = c2d(sys_ss, ts*maxDelayFactor);
                        MPCobjXmax = mpc(sys_max);
                        MPCobjXmax.W.OutputVariables = Q;
                        MPCobjXmax.W.ManipulatedVariables = R;
                        MPCobjXmax.ControlHorizon = 2;
                        MPCobjXmax.PredictionHorizon = 50;
                        MPCobjXmax.MV.Min = -5;
                        MPCobjXmax.MV.Max = 5;
                        MPCobjXmax.MV.RateMin = -2;
                        MPCobjXmax.MV.RateMax = 2;
                        state_mpc_Xmax = mpcstate(MPCobjXmax);

                        MPCobjYmax = mpc(sys_max);
                        MPCobjYmax.W.OutputVariables = Q;
                        MPCobjYmax.W.ManipulatedVariables = R;
                        MPCobjYmax.ControlHorizon = 2;
                        MPCobjYmax.PredictionHorizon = 50;
                        MPCobjYmax.MV.Min = -5;
                        MPCobjYmax.MV.Max = 5;
                        MPCobjYmax.MV.RateMin = -2;
                        MPCobjYmax.MV.RateMax = 2;
                        state_mpc_Ymax = mpcstate(MPCobjYmax);
                        state_mpc_Xmax.plant = x_X;
                        state_mpc_Ymax.plant = x_Y;
                        % control input based on estimated delay
                        [u_Xmax, info_Xmax] = mpcmove(MPCobjXmax,state_mpc_Xmax,y_X(1),r_X);
                        [u_Ymax, info_Ymax] = mpcmove(MPCobjYmax,state_mpc_Ymax,y_Y(1),r_Y);
                        risk = sqrt(((estimatedDelayFactor - delayFactor)/maxDelayFactor)^2);
                        u_X = u_Xmax + (1 - risk) * (u_Xe-u_Xmax);
                        u_Y = u_Ymax + (1 - risk) * (u_Ye-u_Ymax);   
                        end
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
            delayCounter = delayCounter + 1;
            delayCounter = mod(delayCounter,delayFactor);
         end
       
            
        chatpub = rospublisher('/cmd_acc','geometry_msgs/Twist');
        msg = rosmessage(chatpub);
        msg.Linear.X = u_X;
        msg.Linear.Y = u_Y;
         if(index <5)
             msg.Linear.X = 0;
             msg.Linear.Y = -0.045;
         end

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
         plot(y_X(1), y_Y(1), 'b--o');
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