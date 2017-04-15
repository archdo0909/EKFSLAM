close all;
clear all;
clc

time = 0;
endtime = 50;
time_ob = 0;
time_vel = 0;
time_yaw = 0;
timeForMv=3;
t_max = 0;
global dt;
global PoseSize;PoseSize=3;
global LMSize;LMSize=2;
global Qsigma
Qsigma = diag([0.1 toradian(25)]).^2;
dt = 0.1;
nSteps = ceil((endtime - time)/dt);
xEst=[0 0 0]';
xTrue=xEst;
xd=xTrue;
LM = [0 1;
      0.5 0.5];
u = [0 0]';  
LM_I = 100000;  
%len = 15;
result.u = [];
result.xTrue = [];
result.t_max = [];
result.IN=[];
result.xEst=[];
cnt=0;
cnt_ob=0;
for i = 1:nSteps
    
    if time_vel >= timeForMv
       if cnt_ob == 0 
           u = control_new(time_vel, time_yaw); 
           [z,xTrue,IN,xd,u,t_max]=Observation_com(xTrue,xd,u,LM,LM_I);
           result.IN=[result.IN; IN];
           time_ob = time_ob + t_max;
           cnt = cnt + 1;
           cnt_ob = cnt_ob + 1;
       else
           if time >= time_ob
               time_vel = 0;
               cnt_ob = 0;
           else
               time = time + dt;
               u = control_new(time_vel, time_yaw);
               %result.u=[result.u; u' time]; 
               result.xTrue = [result.xTrue; xTrue' time];
           end
       end      
    else
       time = time + dt;
       time_ob = time_ob + dt;
       time_vel = time_vel + dt;
       time_yaw = cnt*timeForMv + time_vel; 
       u = control_new(time_vel, time_yaw);
       xTrue = motion(xTrue,u);
       result.xTrue=[result.xTrue; xTrue' time];
       result.u=[result.u; u' time]; 
       %Animation_test(result,LM)
    end
    
    % --------EKF SLAM-----------
    % Predict
    xEst = motion(xEst, u);
    result.xEst = [result.xEst; xEst'];
    
     
    %result.t_max = [result.t_max; t_max time];
    %result.IN=[result.IN; IN];
    %Animation_test(result,LM)
end
%csvwrite('t_max.csv', result.t_max);
csvwrite('u.csv', result.u)
%csvwrite('IN.csv', result.IN)
csvwrite('xTrue.csv', result.xTrue);
csvwrite('xEst.csv', result.xEst);

