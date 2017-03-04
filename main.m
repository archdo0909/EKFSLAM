close all;
clear all;
clc

time = 0;
endtime = 20;
global dt;
dt = 0.1;
nSteps = ceil((endtime - time)/dt);

%Result parameter
result.u=[];
result.xTrue=[];
result.xd=[];
result.xEst=[];
result.uncertainty=[];
result.time=[];
result.error_lm=[];
%state vector [x y yaw]'
xEst = [0 0 0]';
global PoseSize;PoseSize=3;
global LMSize;LMSize=2;

%True State
xTrue = xEst;

%Dead Reckoning State
xd = xTrue;
 
%Covariance Matrix for predict
%R = diag([0.001 0.001 toradian(1.5)]).^2;
R = diag([0.01 0.01 toradian(1.5)]).^2;
%Covariance Matrix for Q
% global Q;
% Q = diag([toradian(15) toradian(30)]).^2;
global Q;
%Q = diag([1.1 toradian(5)]).^2;
Q = toradian(20)^2;
global Qsigma
Qsigma = diag([0.1 toradian(25)]).^2;
%global Rsigma
%Rsigma = diag([0.1 toradian(1)]).^2;
%global Wsigma 
%Wsigma = diag([toradian(1)]);
%number of Landmark
n = 1;
%Landmark position [x y]
LM = [0  10]'; 
MAX_RANGE = 30; 
MAX_ANGLE = 90 * pi/180;
alpha = 1;
%initP=eye(2)*100;
%initP=diag([5 10]);
PEst = zeros(3,3);
len = 15;
global Ssigma
%Ssigma = 10^2;
Ssigma = 10;

tic;
% Main loop
for i = 1: nSteps
   time = time + dt;
   % Input
   u = control(time);
   %Observation
   %[z,xTrue,xd,u]=Observation(xTrue,xd,u,LM,MAX_R NGE);
   [z, xTrue,xd,u]=Observation_com(xTrue,xd,u,LM,len);
   %------ EKF SLAM -------
   % Predict
   xEst = motion(xEst, u);
   [G,Fx]=JacobianF(xEst, u);
   PEst = G'*PEst*G + Fx'*R*Fx;
   
   
   
   %Update      
   %zl=CalcLMPosiFromZ(xEst,z,len);
%    PAug = [PEst zeros((length(xEst)), LMSize);
%             zeros(LMSize, length(xEst)) initP]; 
%     xAug=[xEst;zl];    
%     PEst = PAug;
%     xEst = xAug;
if(length(xEst)==3)
    [PEst, xEst]=Data_Augmentation(z,xEst,PEst,len);
end 
    
    [K,y,H]=Update(z,len,xEst,PEst);
    xEst = xEst + K*y;
    PEst = (eye(size(xEst,1)) - K*H)*PEst;
    
   xEst(3)=PI2PI(xEst(3));
   
   d = LM - xEst(4:5);
   d_error = d'*d;
   error_lm = sqrt(d_error);
   [Area]=RegionofUncertainty(PEst);
   
   result.time=[result.time; time];
   result.xTrue=[result.xTrue; xTrue'];
   result.xd=[result.xd; xd'];
   result.u=[result.u; u];
   result.uncertainty=[result.uncertainty; Area];
   result.xEst=[result.xEst; xEst(1:3)'];
   result.error_lm=[result.error_lm; error_lm];
   %result.u=[result.u; u'];
   
      %Animation (remove some flames)
    if rem(i,5)==0 
        Animation(result,xTrue,LM,z,xEst,PEst);
        %movcount=movcount+1;
        %mov(movcount) = getframe(gcf);% ??????????????????
    end
    %animation2(PEst,time);
    
   
end

toc

DrawGraph(result,xEst,LM,endtime);


