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
endLocation = [12 2];
path =  [ 2.0000    1.0000 0;
    3.5    4.2496 0.2;
    4.2233    8 0.2;
    8.5    8 0.2;
    8.7290    2.9738 0.1;
    8.9361    2.1709 0.1;
   12.0000    2.0000 0.1];
%findpath(prm, startLocation, endLocation);
%safe_x = [3 3 4.5   4.5  12.5 12.5 7.5  7.5 2.5 2.5 1   1]';
%safe_y = [0 2.5 2.5 7     7    12 12 9   9   5.5 5.5 0]';
%more safe boundaries
plot(path(:,1), path(:,2), 'x');
safe_x = [2.75 2.75 4.25   4.25 7.75 7.75 6.25 6.25 12.25 12.25 9.25 9.25 12.25 12.25  9.25 9.25 12.25 12.25 7.75  7.75  3.75 3.75  5.75  5.75  1.25  1.25 2.75 2.75 1.25 1.25 ]';
safe_y = [0    2.75 2.75   7.25 7.25 5.25 5.25 1.25 1.25   2.25 2.25 4.25 4.25    5.25 5.25 7.25 7.25  11.75 11.75 8.75  8.75 10.75 10.75 11.75 11.75 7.25 7.25 5.25 5.25 0  ]';

%safe_x1 = [2.5 2.5 4 4   8   8 6.5 6.5 12 12 9 9   12 12  9 9   12  12   8    8   3.5 3.5  5.5  5.5  1.5  1.5 3   3 1.5 1.5]';
%safe_y1 = [0    3  3 7.5 7.5 5 5   1.5 1.5 2 2 4.5 4.5 5  5 7.5 7.5 11.5 11.5 8.5  8.5 11    11   11.5 11.5 7.5 7.5 5 5 0  ]';

sd = 0.25;
safe_polys_x = [3-sd 3-sd   3   3 3      4.5    4.5 3   4.5-sd 4.5-sd 4.5  4.5    4.5    7+sd 7+sd 4.5 1+sd 1+sd   1      1 1      2.5    2.5 1   2.5+sd 2.5+sd 2.5 2.5    4-sd 7.5+sd 7.5+sd 4-sd];
safe_polys_y = [0 2.5+sd 2.5+sd 0 2.5+sd 2.5+sd 2.5 2.5 2.5+sd 7+sd   7+sd 2.5+sd 2.5+sd 7+sd 7     7  0    5.5-sd 5.5-sd 0 5.5-sd 5.5-sd 5.5 5.5 5.5-sd 7      7   5.5-sd 9-sd 9-sd   9       9];
safe_polys_type = [1 2 1 2 3 4 3 4];
regionsCount = size(safe_polys_type,2);
plot(safe_x,safe_y);
%plot(safe_x1,safe_y1);
%rectangle('Position',[1,2,5,10],'FaceColor',[0 .5 .5],'EdgeColor','b','LineWidth',3)


delay =1;
adaptation =1;
conservative = 0;
delayFactor = 1;
maxDelayFactor = 10;

ts = 0.01;
l = 3;
N = 5;
H = 1;
attackTime = 1;
C = [1 0 0 0;
     0 1 0 0;
     0 0 1 0;
     0 0 0 1];
D = [0 0;
     0 0;
     0 0;
     0 0];
     
states = {'x' 'y' 'theta','gamma'};
inputs = {'v' 'phi'};
outputs = {'x' 'y', 'theta', 'gamma'};

R =    [  1  0;    
         0   1];

Q = [1000  0  0  0;
     0    1000 0 0;
     0    0     1 0;
     0    0     0 0];
 
alpha = 0.7;


attackCounter = 0;
delayCounter = 0;
estimatedDelayFactor = 1;

v = 0.0001;
phi = 0.0001;
x =[2;1;3.14/4;0];
theta = x(3);

time = 0;
index = 1;
for goalIndex = 2 : length(path)
    r_X = path(goalIndex,1);
    r_Y = path(goalIndex,2);
    r = [r_X; r_Y; 3.14/4;0];
    epsilon = path(goalIndex,3);
    distance = norm([r_X - x(1) r_Y - x(2)]);
    while (distance > epsilon)
         A = [0  0  -sin(theta)*v 0;
         0  0   cos(theta)*v 0;
         0  0   0  v*(sec(x(4))^2)/l;
         0  0   0   0];

        B = [cos(theta) 0;
             sin(theta) 0;
             tan(x(4))/l 0;
             0           1];
        % continuous state space
            sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);
        y = C *x;
        
        if(delay == 0)
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
        else if(delayCounter == 0)    
            if(adaptation == 1)
                if(conservative == 1)
                    sys = c2d(sys_ss, ts*maxDelayFactor);
                    MPCobj = mpc(sys);
                    MPCobj.W.OutputVariables = Q;
                    MPCobj.W.ManipulatedVariables = R;
                    MPCobj.MV(1).Min =  0.0;
                    MPCobj.MV(1).Max = 2;
                    MPCobj.MV(1).RateMin = -2;
                    MPCobj.MV(1).RateMax = 2;
                    MPCobj.MV(2).Min = -1.4;
                    MPCobj.MV(2).Max = 1.4 ;
                    x_mpc = mpcstate(MPCobj);
                    x_mpc.plant = x; % used for mpc
                    u = mpcmove(MPCobj,x_mpc,y,r)
                else
                    % non conservative - Adaptive algorithm
                    %estimate delay 
                    estimatedDelayFactor = (1-alpha)*estimatedDelayFactor + alpha *delayFactor;
                    delayPlot(index) = (delayFactor-1)*100;
                    estimatedDelayPlot(index) = (estimatedDelayFactor-1)*100;
                    %Adaptation
                    deltaDelay = (maxDelayFactor - estimatedDelayFactor)/N;
                    if(sum(inpolygon(x(1),x(2),safe_x,safe_y)) ~= 1)
                            % We are inside the unsafe region now, apply the
                            % conservative control inputs
                        sys = c2d(sys_ss, ts*maxDelayFactor);
                        MPCobj = mpc(sys);
                        MPCobj.W.OutputVariables = Q;
                        MPCobj.W.ManipulatedVariables = R;
                        MPCobj.ControlHorizon = 2;
                        MPCobj.PredictionHorizon = 100;
                        MPCobj.MV(1).Min =  0.0;
                        MPCobj.MV(1).Max = 2;
                        MPCobj.MV(1).RateMin = -2;
                        MPCobj.MV(1).RateMax = 2;
                        MPCobj.MV(2).Min = -1.4;
                    MPCobj.MV(2).Max = 1.4 ;
                        % add the constraint to get out of the inflated region
%                         for regionsCounter = 0: regionsCount-1
%                             if(sum(inpolygon(x(1),x(2),safe_polys_x((1+4*regionsCounter):(4+4*regionsCounter)),safe_polys_y((1+4*regionsCounter):(4+4*regionsCounter)))) == 1)
%                                 if(safe_polys_type(regionsCounter+1)== 1)
%                                     MPCobj.OV(1).Max = safe_polys_x(1+4*regionsCounter);
%                                 elseif (safe_polys_type(regionsCounter+1) == 2)
%                                     MPCobj.OV(2).Min = safe_polys_y(1+4*regionsCounter);
%                                 elseif (safe_polys_type(regionsCounter+1) == 3)
%                                     MPCobj.OV(1).Min = safe_polys_x(1+4*regionsCounter);
%                                 else
%                                     MPCobj.OV(2).Max = safe_polys_y(1+4*regionsCounter);
%                                 end
%                             end
%                         end
                        Y = struct('Min',[-Inf,-Inf,-Inf,-Inf],'Max',[2.75,Inf,Inf,Inf]);
                        U = struct('Min',[-Inf,-Inf]);
                        setterminal(MPCobj,Y,U);
                        x_mpc = mpcstate(MPCobj);
                        x_mpc.plant = x; % used for mpc
                        review(MPCobj);
                        u = mpcmove(MPCobj,x_mpc,y,r);
                    else
                        while(1)     
                            sys_e = c2d(sys_ss, ts*estimatedDelayFactor);
                            sys_max = c2d(sys_ss, ts*maxDelayFactor);
                            MPCobj = mpc(sys_e);
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
                            [u_e, info_e] = mpcmove(MPCobj,x_mpc,y,r);
                            if(estimatedDelayFactor >= maxDelayFactor)
                                u = u_e;
                                break;
                            end
                            x_Xmax = zeros(4,H);
                            x_Xmax(:,1) = x;
                            for h = 1:H
                                % next states if Ue is applied and maximum delay occurs
                                x_Xmax(:,h+1) =  sys_max.A * x_Xmax(:,h) + sys_max.B * info_e.Uopt(h,:)';
                            end
                            % Check if series of Uopt is safe if applied for delta_max or not
                            safeCount = sum(inpolygon(x_Xmax(1,:),x_Xmax(2,:),safe_x,safe_y));
                            if (safeCount == H + 1)
                                % Ue is safe
                                u = u_e;
                                break;
                            else
                                estimatedDelayFactor = estimatedDelayFactor + deltaDelay;
                            end
                        end
                    end
                end
            else % no adaptation
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
            end
%             if(mod(index,20) == 0)
%                 if (delayFactor == maxDelayFactor)
%                     delayFactor = 1;
%                 else
%                     delayFactor = maxDelayFactor;
%                 end
%             end
%             index = index + 1;
             delayFactor = maxDelayFactor;
              % delayFactor =  randi([1 maxDelayFactor],1,1);
            end
            delayCounter = delayCounter + 1;
            delayCounter = mod(delayCounter,delayFactor);
        end
        [s,x] = ode45(@(t,x) robotModel(x,u(1),u(2)),[0 ts],x);
        x = x(end,:)';
        plot(x(1), x(2), 'b--o');
        distance = norm([r_X - x(1) r_Y - x(2)]);
        time = time + ts;
        v = u(1);
        phi = u(2);
        theta = x(3);
    end
end
time
title('');
%title(strcat('Random Delays with adaptation algorithm. Time = ', num2str(time)));
hold off;
if(delay == 1 && adaptation == 1 &&conservative~=1)
    figure
    hold on
    plot(estimatedDelayPlot,'LineWidth',2);
    xlabel('Controller time steps');
    ylabel('Controller Overhead %');
    plot(delayPlot,'LineWidth',1);
    legend('Estimated Overhead', 'Actual Overhead');
end
