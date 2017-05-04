close all;
clear all;
clc
%centimeter scale
time = 0;
endtime = 20;
xEst=[0 0 0]';
global dt;
global PoseSize;PoseSize=length(xEst);
global LMSize;LMSize=2;
dt = 0.1;
nSteps = ceil((endtime - time)/dt);
%True State
xTrue=xEst;
%Dead Reckoning State
xd=xTrue;

LM = [4 4;
      2 4;
      2 2;
      4 2;
      3 3];
u = [0 0]';  
PEst = eye(3);
initP = eye(2)*1000;
%LM_I = 100000; 
LM_I = Cal_Intensity_Area(10, 6); % cm, m
%len = 15;
result.u = [];
result.xTrue = [];
result.z = [];
result.xd = [];
result.uncertainty=[];
result.PEst=[];
result.xEst=[];
result.mdist=[];
R = diag([0.2 0.2 toradian(1)]).^2;
global Q;     %CalcInno_Multicom, Augmented_data
Q = diag([0.1 toradian(25)]).^2;
%Q = toradian(25).^2;

global Qsigma  %Observation_Multicom in Process Noise
Qsigma = diag([0.1 toradian(25)]).^2;
global Rsigma  %Observation_Multicom in noise 
Rsigma=diag([0.1 toradian(1)]).^2;
global Ssigma
Ssigma = 0.1^2;
alpha = 1;
for i = 1:nSteps
    
    time = time + dt;
    
    u = control_new(time);
    
    xTrue = motion(xTrue, u);
    u = u + Qsigma*randn(2,1);
    xd = motion(xd, u); %Dead Reckoning
     
    [z]=Observation_MultiCom(xTrue,u,LM,LM_I);
    
    %--------EKF SLAM----------
    % Predict
    xEst = motion(xEst, u);
    [G,Fx]=JacobianF(xEst,u);
    PEst = G'*PEst*G + Fx'*R*Fx;

    %Update
    for iz=1:length(z(:,1))
         [PAug, xAug]=Augmented_data_Multicom(z(iz,:),xEst,PEst,LM_I);
%         zl=CalcRSPosiFromZ(xEst,z(iz,:),LM_I);
%         xAug=[xEst;zl];
%         PAug=[PEst zeros(length(xEst),LMSize);
%               zeros(LMSize,length(xEst)) initP];
          
        mdist=[];
        for il=1:GetnLM(xAug)
            if il==GetnLM(xAug)
                mdist=[mdist alpha];
            else
                lm=xAug(4+2*(il-1):5+2*(il-1));
                [y,S,H]=CalcInno_Multicom(lm,xAug,PAug,z(iz,1:2),il,LM_I);
                %mdistance=y'*inv(S)*y;
                %disp(mdistance);
                mdist=[mdist y'*inv(S)*y];
            end
        end
        [C,I]=min(mdist);
       
        if I==GetnLM(xAug)
            %disp(I);
            xEst=xAug;
            PEst=PAug;
        end
        
            lm=xEst(4+2*(I-1):5+2*(I-1));
            [y,S,H]=CalcInno_Multicom(lm,xEst,PEst,z(iz,1:2),I,LM_I);
            K = PEst*H'*inv(S);
            xEst = xEst + K*y;
            PEst = (eye(size(xEst,1)) - K*H)*PEst;
    end
    
    xEst(3)=PI2PI(xEst(3));
     
    result.xTrue=[result.xTrue; xTrue'];
    result.xd=[result.xd; xd'];
    result.u = [result.u; u'];
    result.z = [result.z; z];
    result.xEst=[result.xEst; xEst(1:3)'];
    
    %Animation(result,xTrue,LM,z,xEst,zl);
    Animation2(result,LM,xEst,PEst);
   
    
end
DrawGraph(result,LM);
%csvwrite('mdist.csv', result.mdist);
%csvwrite('u.csv', result.u);
%csvwrite('z.csv', result.z);
%csvwrite('xTrue.csv', result.xTrue);
% csvwrite('xEst.csv', result.xEst);
%DrawGraph_test(result,LM);
function Animation(result,xTrue,LM,z,xEst,zl)
hold off;
plot(result.xTrue(:,1),result.xTrue(:,2),'.b');hold on;
plot(LM(:,1),LM(:,2),'pk','MarkerSize',10);hold on;

% if~isempty(z)
%    for iz=1:length(z(:,1))
%        ray=[xTrue(1:2)';z(iz,3:4)]; 
%        plot(ray(:,1),ray(:,2),'-r');hold on; 
%    end
% end

for il=1:GetnLM(xEst)
    plot(xEst(4+2*(il-1)),xEst(5+2*(il-1)),'.c');hold on;
end
plot(zl(1,:),zl(2,:),'.b');hold on;
plot(result.xd(:,1),result.xd(:,2),'.k');hold on;
plot(result.xEst(:,1),result.xEst(:,2),'.r');hold on;
arrow=0.5;
x=result.xEst(end,:);
quiver(x(1),x(2),arrow*cos(x(3)),arrow*sin(x(3)),'ok');hold on;
axis equal;
grid on;

drawnow;
end
function Animation2(result,LM,xEst,PEst)
hold off;
plot(result.xTrue(:,1),result.xTrue(:,2),'.b');hold on;
ShowErrorEllipse(xEst,PEst);
for il=1:GetnLM(xEst)
    plot(xEst(4+2*(il-1)),xEst(5+2*(il-1)),'Diamond');hold on;
    ShowErrorEllipse_LM(xEst, PEst(4+2*(il-1):5+2*(il-1),4+2*(il-1):5+2*(il-1)),il);
end
 
plot(LM(:,1),LM(:,2),'pk','MarkerSize',10);hold on;
plot(result.xd(:,1),result.xd(:,2),'.k');hold on;
plot(result.xEst(:,1),result.xEst(:,2),'.r');hold on;
axis equal;
grid on;

drawnow;
end
function DrawGraph(result,LM)
figure(2);
hold off;
x=[ result.xTrue(:,1:2) result.xEst(:,1:2)];
%x=[result.xTrue(:,1) result.xTrue(:,2)];
set(gca, 'fontsize', 16, 'fontname', 'times');
plot(x(:,1), x(:,2),'-b','linewidth', 4); hold on;
plot(result.xd(:,1), result.xd(:,2),'-k','linewidth', 2); hold on;
plot(x(:,3), x(:,4),'-r','linewidth', 4); hold on;
plot(LM(:,1),LM(:,2),'pk','MarkerSize',10);hold on;
% for il=1:GetnLM(xEst)
%     plot(xEst(4+2*(il-1)),xEst(5+2*(il-1)),'Diamond');hold on;
% end
 
title('EKF SLAM Result', 'fontsize', 16, 'fontname', 'times');
xlabel('X (m)', 'fontsize', 16, 'fontname', 'times');
ylabel('Y (m)', 'fontsize', 16, 'fontname', 'times');
legend('Ground Truth','Dead Reckoning','EKF SLAM','True LM','Estimated LM');
grid on;
axis equal;

end


