close all;
clear all;
clc

time = 0;
endtime = 6.0;
time_ob = 0;
time_vel = 0;
time_yaw = 0;
timeForMv=3;
t_max = 0;
global dt;
global PoseSize;PoseSize=3;
global LMSize;LMSize=2;
dt = 0.1;
nSteps = ceil((endtime - time)/dt);
xEst=[0 0 0]';
xTrue=xEst;
xd=xTrue;
% LM = [0 1;
%       0.5 0.5];
LM = [0 1];
u = [0 0]';  
PEst = eye(3);
initP = eye(2)*1000;
LM_I = 1000;  
%len = 15;
result.u = [];
result.xTrue = [];
result.t_max = [];
result.xd = [];
result.uncertainty=[];
result.PEst=[];
result.IN=[];
result.xEst=[];
z=[];
cnt=0;
cnt_ob=0;
alpha = 1;
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
    
%     if time_vel >= timeForMv
%        if cnt_ob == 0 
%            u = control_new(time_vel); 
%            [z,xTrue,IN,xd,u,t_max]=Observation_MultiCom(xTrue,z,xd,u,LM,LM_I);
%            %result.IN=[result.IN; IN];
%            time_ob = time_ob + t_max;
%            cnt = cnt + 1;
%            cnt_ob = cnt_ob + 1;
%        else
%            if time >= time_ob
%                time_vel = 0;
%                cnt_ob = 0;
%            else
%                time = time + dt;
%                u = control_new(time_vel);
%                %result.u=[result.u; u' time]; 
%                %result.xTrue = [result.xTrue; xTrue' time];
%            end
%        end      
%     else
%        time = time + dt;
%        time_ob = time_ob + dt;
%        time_vel = time_vel + dt;
%        time_yaw = cnt*timeForMv + time_vel; 
%        u = control_new(time_vel);
%        xTrue = motion(xTrue,u);
%        %result.xTrue=[result.xTrue; xTrue' time];
%        %result.u=[result.u; u' time]; 
%     end
    
    % --------EKF SLAM-----------
    % Predict
%     xEst = motion(xEst, u);
%     %result.xEst = [result.xEst; xEst'];
%     [G,Fx]=JacobianF(xEst, u);
%     PEst = G'*PEst*G + Fx'*R*Fx;
%     
%     
%     % Update
%     if isempty(z) == 0
%         for iz=1:length(z(:,1))
%             zl=CalcRSPosiFromZ(xEst,z(iz,:));
%             xAug=[xEst;zl];
%             PAug=[PEst zeros(length(xEst), LMSize);
%                   zeros(LMSize, length(xEst)) initP];
% 
%             mdist=[]; %The list for Mahalanobis distance
%             for il=1:GetnLM(xAug)
%                 if il==GetnLM(xAug)
%                     mdist=[mdist alpha];
%                 else
%                     lm=xAug(4+2*(il-1):5+2*(il-1));
%                     [y,S,H]=CalcInnovation(lm,xAug,PAug,z(iz,1:2),il);
%                     mdist=[mdist y'*inv(S)*y];
%                 end
%             end    
% 
%             [C, I] = min(mdist);
% 
%             if I==GetnLM(xAug)6
%                %disp("New LM")
%                xEst=xAug;
%                PEst=PAug;
%             end
% 
%             lm=xEst(4+2*(I-1):5+2*(I-1));
%             [y,S,H]=CalcInnovation(lm,xEst,PEst,z(iz,1:2),I);
%             K = PEst*H'*inv(S);
%             xEst=xEst+K*y;
%             PEst=(eye(size(xEst,1)) - K*H)*PEst;
%         end
%     end
%     
%     xEst(3)=PI2PI(xEst(3));
%     result.xEst=[result.xEst; xEst(1:3)' time];
    result.xTrue=[result.xTrue; xTrue' time];
    result.u = [result.u; u' time];
    %result.t_max = [result.t_max; t_max time];
    %result.IN=[result.IN; IN];
    Animation_test(result)
end
%csvwrite('t_max.csv', result.t_max);
csvwrite('u.csv', result.u)
% %csvwrite('IN.csv', result.IN)
 csvwrite('xTrue.csv', result.xTrue);
% csvwrite('xEst.csv', result.xEst);
DrawGraph_test(result,LM);


