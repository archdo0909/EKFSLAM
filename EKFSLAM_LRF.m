close all;
clear all;
clc
%centimeter scale
time = 0;
endtime = 24;
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
result.time = [];
result.PEst=[];
result.xEst=[];
result.mdist=[];
result.xEst_up=[];
result.error_robot = [];
result.error_robot_dead = [];
result.uncertainty=[];
%result.PEst_up=[];
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
len = 3;
MAX_RANGE = 20;
alpha = 1;
time_ob = 0;
for i = 1:nSteps
    
    time = time + dt;
    time_ob = time_ob + dt;
    
    u = control_new(time);

    xTrue = motion(xTrue, u);
    u = u + Qsigma*randn(2,1);
    xd = motion(xd, u); %Dead Reckoning
    
     % Predict
    xEst = motion(xEst, u);
    [G,Fx]=JacobianF(xEst,u);
    PEst = G'*PEst*G + Fx'*R*Fx;
    z=[];
    %if (time_ob >= 0.1)
        
    %    time_ob = 0;
        [z]=Observation_LRF(xTrue,u,LM,MAX_RANGE);
        %[z]=Observation_multi_bearing(x,u,LM);

        %--------EKF SLAM----------
    %     % Predict
    %     xEst = motion(xEst, u);
    %     [G,Fx]=JacobianF(xEst,u);
    %     PEst = G'*PEst*G + Fx'*R*Fx;

        %Update
        for iz=1:length(z(:,1))
             [PAug, xAug]=Augmented_data_Multicom(z(iz,:),xEst,PEst,LM_I);
             %[PAug, xAug]=Augmented_data_Multibearing(z(iz,:),xEst,PEst,LM_I,len);
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
                    [y,S,H]=CalcInno_LRF(lm,xAug,PAug,z(iz,1:2),il);
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
                [y,S,H]=CalcInno_LRF(lm,xEst,PEst,z(iz,1:2),I);
                K = PEst*H'*inv(S);
                xEst = xEst + K*y;
                PEst = (eye(size(xEst,1)) - K*H)*PEst;
                xEst(3)=PI2PI(xEst(3));
                %result.xEst_up = [result.xEst_up; xEst(1:3)'];
        end
    %end
    xEst(3)=PI2PI(xEst(3));
    
    d_robot = xTrue(1:2)-xEst(1:2);
    d_robot_error = d_robot'*d_robot;
    d_robot_dead = xTrue(1:2)-xd(1:2);
    d_robot_deaderror= d_robot_dead'*d_robot_dead;
    error_robot = sqrt(d_robot_error);
    error_robot_dead = sqrt(d_robot_deaderror);
    result.error_robot = [result.error_robot; error_robot];
    result.error_robot_dead = [result.error_robot_dead; error_robot_dead];
    [Area]=RegionofUncertainty(PEst);
    result.uncertainty=[result.uncertainty; Area];
    
    result.time = [result.time; time];
    result.xTrue=[result.xTrue; xTrue'];
    result.xd=[result.xd; xd'];
    result.u = [result.u; u'];
    result.z = [result.z; z];
    result.xEst=[result.xEst; xEst(1:3)' time];
    
    %Animation(result,xTrue,LM,z,xEst,zl);
    Animation2(result,LM,xEst,PEst,time);
   
    
end
csvwrite('result_xEst.csv',result.xEst);
DrawGraph(result,LM,xEst,endtime);
DrawGraph_no(result, LM,xEst);
Total_d=Cal_error_LM(xEst, LM)
%csvwrite('mdist.csv', result.mdist);
%csvwrite('u.csv', result.u);
%csvwrite('z.csv', result.z);
%csvwrite('xTrue.csv', result.xTrue);
% csvwrite('xEst.csv', result.xEst);
%DrawGraph_test(result,LM);
function [z]=Observation_LRF(x,u,LM,MAX_RANGE)
      

      global Rsigma;

      z=[];
      
    for iz=1:length(LM(:,1))
      yaw = zeros(3,1);
      yaw(3)=-x(3);
      %localLM = HomogeneousTransformation2D(LM(1:2)'-x(1:2)',yaw');
      localLM = HomogeneousTransformation2D(LM(iz,:)-x(1:2)',yaw');
      d = norm(localLM);

      % 8cm * 8cm = 0.08 * 0.08 = 0.08 * 0.08 m^2 = 0.0064
      % 4 * pi * d ^ 24
      if d<MAX_RANGE
         noise = Rsigma*randn(2,1);
         z = [z; [d+noise(1) PI2PI(atan2(localLM(2),localLM(1)) + noise(2)) LM(iz,:)]]
      end
    end
      
end   
function [y,S,H]=CalcInno_LRF(lm,xEst,PEst,z,LMID)

global Q;

delta=lm-xEst(1:2);
q=delta'*delta;
zangle=atan2(delta(2),delta(1))-xEst(3);

zp = [sqrt(q) PI2PI(zangle)];
y=(z-zp)';

H=jacobH(q,delta,xEst,LMID);
S=H*PEst*H'+Q;
end 
function Animation(result,xTrue,LM,z,xEst,zl)
hold off;
plot(result.xTrue(:,1),result.xTrue(:,2),'.b');hold on;
plot(LM(:,1),LM(:,2),'pk','MarkerSize',10);hold on;

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
function Animation2(result,LM,xEst,PEst,time)
figure(1);
hold off;
plot(result.xTrue(:,1),result.xTrue(:,2),'.b');hold on;
ShowErrorEllipse(xEst,PEst);
for il=1:GetnLM(xEst)
    plot(xEst(4+2*(il-1)),xEst(5+2*(il-1)),'Diamond');hold on;
    %ShowErrorEllipse_LM(xEst, PEst(4+2*(il-1):5+2*(il-1),4+2*(il-1):5+2*(il-1)),il);
end
 
plot(LM(:,1),LM(:,2),'pk','MarkerSize',10);hold on;
plot(result.xd(:,1),result.xd(:,2),'.k');hold on;
plot(result.xEst(:,1),result.xEst(:,2),'.r');hold on;
axis equal;
grid on;
    
%     if(time >= 0.2 && time <= 0.3)
%         saveas(figure(1), 'figure_0.fig')
%     elseif (time >= 6 && time <= 6.1)
%         saveas(figure(1), 'figure_6.fig')
%     elseif (time >= 12 && time <= 12.1)
%         saveas(figure(1), 'figure_12.fig')
%     elseif (time >= 18 && time <= 18.1)
%         saveas(figure(1), 'figure_18.fig')
%     elseif (time >= 23.9 && time <= 24)
%         saveas(figure(1), 'figure_24.fig')
%     end
    

drawnow;
end
function DrawGraph(result,LM,xEst,endtime)
figure(2);
hold off;
x=[ result.xTrue(:,1:2) result.xEst(:,1:2)];
%x=[result.xTrue(:,1) result.xTrue(:,2)];
set(gca, 'fontsize', 16, 'fontname', 'times');
plot(x(:,1), x(:,2),'-b','linewidth', 3); hold on;
%plot(result.xTrue(:,1), result.xTrue(:,2),'-b','linewidth', 4);hold on;
plot(result.xd(:,1), result.xd(:,2),'-k','linewidth', 2); hold on;
plot(x(:,3), x(:,4),'-r','linewidth', 3); hold on;
%plot(result.xEst_up(:,1), result.xEst_up(:,2),'-r','linewidth', 4); hold on;
plot(LM(:,1),LM(:,2),'pk','MarkerSize',10);hold on;
for il=1:GetnLM(xEst)
    plot(xEst(4+2*(il-1)),xEst(5+2*(il-1)),'Diamond');hold on;
end
 
%title('EKF SLAM Result', 'fontsize', 16, 'fontname', 'times');
xlabel('X (m)', 'fontsize', 16, 'fontname', 'times');
ylabel('Y (m)', 'fontsize', 16, 'fontname', 'times');
legend('Ground Truth','Dead Reckoning','EKF SLAM','True LM','Estimated LM');
grid on;
axis equal;

figure(6);
set(gca, 'fontsize', 16, 'fontname', 'times');
plot(result.time(:), result.error_robot(:),'-r', 'linewidth', 2); hold on;
plot(result.time(:), result.error_robot_dead(:),'-g','linewidth', 2); hold on;
title('Error on the estimates of robot position','fontsize', 16, 'fontname', 'times');
xlabel('Iteration','fontsize', 16, 'fontname', 'times');
ylabel('Error (m)', 'fontsize', 16, 'fontname', 'times');
legend('SLAM','Dead reckoning','fontsize');
grid on;
axis equal;

figure(7);
%set(gca, 'fontsize', 16, 'fontname', 'times');
plot(result.time(:),result.uncertainty(:),'-r','linewidth',1); hold on;
set(gca, 'XTick', [1:1:endtime]);
title('Uncertainty regions of robot position','fontsize', 16, 'fontname', 'times');
xlabel('time (sec)','fontsize', 16, 'fontname', 'times');
ylabel('Uncertainty (m^2)', 'fontsize', 16, 'fontname', 'times');
grid on;
axis equal;


end
function [PAug, xAug]=Augmented_data_Multicom(z,xEst,PEst,LM_I)
    
    global Q
    global Ssigma
    %initP=eye(2)*1000;

    zl = xEst(1:2)+[z(1)*cos(xEst(3)+z(2)); z(1)*sin(xEst(3)+z(2))];
    
%%%%% Normal setting
    Yx = [1 0 -z(1)*sin(xEst(3)+z(2));
          0 1 z(1)*cos(xEst(3)+z(2))];
    Yz = [cos(z(2)+xEst(3))  -z(1)*sin(z(2)+xEst(3));
          sin(z(2)+xEst(3))  z(1)*cos(z(2)+xEst(3))];
       
    Px_y = PEst(1:3,1:3);  
      
    if(GetnLM(xEst) >= 1)
        for i=1:GetnLM(xEst)
            Px_y = [Px_y PEst(1:3,4+2*(i-1):5+2*(i-1))]; %(3 * 3 or 5 or 7...)
        end 
    end
   Px = Yx*Px_y;  
   
    Ppp = Yx*PEst(1:3,1:3)*Yx'+Yz*Q*Yz';

    PAug = [PEst Px';
            Px Ppp];    
    xAug = [xEst; zl];    
end

function DrawGraph_no(result,LM,xEst)
figure(3);
hold off;
x=[ result.xTrue(:,1:2) result.xEst(:,1:2)];
%x=[result.xTrue(:,1) result.xTrue(:,2)];
set(gca, 'fontsize', 16, 'fontname', 'times');
plot(x(:,1), x(:,2),'-b','linewidth', 3); hold on;
%plot(result.xTrue(:,1), result.xTrue(:,2),'-b','linewidth', 4);hold on;
plot(result.xd(:,1), result.xd(:,2),'-k','linewidth', 2); hold on;
plot(x(:,3), x(:,4),'-r','linewidth', 3); hold on;
%plot(result.xEst_up(:,1), result.xEst_up(:,2),'-r','linewidth', 4); hold on;
plot(LM(:,1),LM(:,2),'pk','MarkerSize',10);hold on;
for il=1:GetnLM(xEst)
    plot(xEst(4+2*(il-1)),xEst(5+2*(il-1)),'Diamond');hold on;
end
 
%title('EKF SLAM Result', 'fontsize', 16, 'fontname', 'times');
xlabel('X (m)', 'fontsize', 16, 'fontname', 'times');
ylabel('Y (m)', 'fontsize', 16, 'fontname', 'times');

grid on;
axis equal;

end


