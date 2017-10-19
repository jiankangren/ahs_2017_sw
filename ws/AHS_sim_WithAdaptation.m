close all;
clearvars;

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
prm.ConnectionDistance = 5;
startLocation = [2 1];
endLocation = [12 10];
path =  [2.0000    1.0000 0;
    2.5246    1.3943 0.5;
    2.7836    3.5884 0.2;
    3.0394    6.8319 0.5;
    4.9519    7.4415 0.5;
    9.2191    8.8708 0.2;
   12.1692   10.0144 0.4;
   12.0000   10.0000 0.1];%findpath(prm, startLocation, endLocation);
%safe_x = [3 3 4.5   4.5  12.5 12.5 7.5  7.5 2.5 2.5 1   1]';
%safe_y = [0 2.5 2.5 7     7    12 12 9   9   5.5 5.5 0]';
%more safe boundaries
plot(path(:,1), path(:,2), 'x');
safe_x = [2.75 2.75 4.25   4.25 12.25 12.25 7.75  7.75 2.75 2.75 1.25    1.25]';
safe_y = [0.25    2.75 2.75   7.25  7.25 11.75 11.75 8.75  8.75  5.25 5.25 0.25]';
plot(safe_x,safe_y);
x_X =[2;0];
x_Y =[1;0];

ts = 0.01;
%t = [0:ts:15];
%N = length(t);
y_X = zeros(1,2);
y_Y = zeros(1,2);
delayFactor = 15;
maxDelayFactor = 20;
N = 10;
H = 20;
attackTime = 1;
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

sys = c2d(sys_ss, ts*maxDelayFactor);
MPCobj_X = mpc(sys);
MPCobj_X.W.OutputVariables = Q;
MPCobj_X.W.ManipulatedVariables = R;
MPCobj_X.ControlHorizon = 2;
MPCobj_X.PredictionHorizon = 50;
MPCobj_X.MV.Min = -5;
MPCobj_X.MV.Max = 5;
MPCobj_X.MV.RateMin = -2;
MPCobj_X.MV.RateMax = 2;
state_mpc_X = mpcstate(MPCobj_X);

MPCobjY = mpc(sys);
MPCobjY.W.OutputVariables = Q;
MPCobjY.W.ManipulatedVariables = R;
MPCobjY.ControlHorizon = 2;
MPCobjY.PredictionHorizon = 50;
MPCobjY.MV.Min = -5;
MPCobjY.MV.Max = 5;
MPCobjY.MV.RateMin = -2;
MPCobjY.MV.RateMax = 2;
state_mpc_Y = mpcstate(MPCobjY);

%review(MPCobj);

delay = 1;
attackCounter = 0;
delayCounter = 0;
estimatedDelayFactor = 1;
%u_X(1) = 0;
%u_Y(1) = 0;
u_X = 0;
u_Y = 0;
time = 0;
for goalIndex = 2 : length(path)
    
    r_X = path(goalIndex,1);
    r_Y = path(goalIndex,2);
    epsilon = path(goalIndex,3);
    distance = norm([r_X - y_X(1) r_Y - y_Y(1)]);
    while (distance > epsilon)
        y_X = x_X;
        y_Y = x_Y;
        if(delay == 0)
             state_mpc_X.plant = x_X;
             u_X = mpcmove(MPCobj_X,state_mpc_X,y_X(1),r_X);
             u_Y = mpcmove(MPCobjY,state_mpc_Y, y_Y(1),r_Y);
        else 
            if(delayCounter == 0)
                %estimate delay TODO: add EWMA
                estimatedDelayFactor = delayFactor;
                %Adaptation
                deltaDelay = (maxDelayFactor - estimatedDelayFactor)/N;
                while(1)     
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
                    if(estimatedDelayFactor >= maxDelayFactor)
                        u_X = u_Xe;
                        u_Y = u_Ye;
                        break;
                    end
                    x_Xmax = zeros(2,H);
                    x_Ymax = zeros(2,H);
                    x_Xmax(:,1) = x_X;
                    x_Ymax(:,1) = x_X;
                    for h = 1:H
                        % next states if Ue is applied and maximum delay occurs
                        x_Xmax(:,h+1) =  sys.A * x_Xmax(:,h) + sys.B * info_Xe.Uopt(h);
                        x_Ymax(:,h+1) =  sys.A * x_Ymax(:,h) + sys.B * info_Ye.Uopt(h);
                    end
                    % Check if series of Uopt is safe if applied for delta_max or not
                    safeCount = sum(inpolygon(x_Xmax(1,:),x_Ymax(1,:),safe_x,safe_y));
                    if (safeCount == H + 1)
                        % Ue is safe
                        u_X = u_Xe;
                        u_Y = u_Ye;
                        break;
                    else
                        estimatedDelayFactor = estimatedDelayFactor + deltaDelay;
                    end
                end
                delayFactor = randi([1 maxDelayFactor],1,1);
            else
             % u_X(i) = u_X(i-1);
             % u_Y(i) = u_Y(i-1);
            end
            delayCounter = delayCounter + 1;
            delayCounter = mod(delayCounter,delayFactor);
        end
        [s,x_X] = ode45(@(t,x_X) doubleIntModel(x_X,u_X),[0 ts],x_X);
        x_X = x_X(end,:)';
        [s,x_Y] = ode45(@(t,x_Y) doubleIntModel(x_Y,u_Y),[0 ts],x_Y);
        x_Y = x_Y(end,:)';
        distance = norm([r_X - y_X(1) r_Y - y_Y(1)]);
        plot(y_X(1), y_Y(1), 'o');
        time = time + ts;
    end
end
time