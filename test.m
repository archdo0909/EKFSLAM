close all;
clear all;
clc

time = 0;
endtime = 3.5;
global dt;
global PoseSize;PoseSize=3;
global LMSize;LMSize=2;
dt = 0.1;
nSteps = ceil((endtime - time)/dt);
xEst=[0 0 0]';
xTrue=xEst;
xd=xTrue;
LM = [1 2;
      2 2];
%LM = [0 1];
u = [0 0]';  
PEst = eye(3);
initP = eye(2)*1000;
LM_I = 100000;  
%len = 15;
result.u = [];
result.xTrue = [];
result.z = [];
result.xd = [];
result.uncertainty=[];
result.PEst=[];
result.xEst=[];
R = diag([0.01 0.01 toradian(1.5)]).^2;
global Q;
%Q = diag([1.1 toradian(5)]).^2;
Q = toradian(20)^2;
global Qsigma
Qsigma = diag([0.1 toradian(25)]).^2;
PEst = zeros(3,3);
global Ssigma
%Ssigma = 10^2;
Ssigma = 10;
global Threshold_Dis
Threshold_Dis = 1.5;
for i = 1:nSteps
    
    time = time + dt;
    
    u = control_new(time);
    
    xTrue = motion(xTrue, u);
    u = u + Qsigma*randn(2,1);
    xd = motion(xd, u); %Dead Reckoning
    
    [z]=Observation_MultiCom(xTrue,u,LM,LM_I,time);
    
    
    result.xTrue=[result.xTrue; xTrue' time];
    result.u = [result.u; u' time];
    result.z = [result.z; z];
    %Animation_test(result)
end
%csvwrite('t_max.csv', result.t_max);
csvwrite('u.csv', result.u);
csvwrite('z.csv', result.z);
csvwrite('xTrue.csv', result.xTrue);
% csvwrite('xEst.csv', result.xEst);
%DrawGraph_test(result,LM);


