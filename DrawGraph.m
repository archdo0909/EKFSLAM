function DrawGraph(result,xEst,LM,endtime)
%Plot Result
 figure(2);
hold off;
x=[ result.xTrue(:,1:2) result.xEst(:,1:2)];
set(gca, 'fontsize', 16, 'fontname', 'times');
plot(x(:,1), x(:,2),'-b','linewidth', 2); hold on;
plot(result.xd(:,1), result.xd(:,2),'-k','linewidth', 2); hold on;
plot(x(:,3), x(:,4),'-r','linewidth', 2); hold on;
plot(LM(1),LM(2),'pk','MarkerSize',10);hold on;

for il=1:GetnLM(xEst)
    plot(xEst(4+2*(il-1)),xEst(5+2*(il-1)),'diamond','MarkerSize',10);hold on;
    %plot(xEst(4+2*(il-1)),xEst(5+2*(il-1)),'.g');hold on;
end
 
title('EKF SLAM Result', 'fontsize', 16, 'fontname', 'times');
xlabel('X (m)', 'fontsize', 16, 'fontname', 'times');
ylabel('Y (m)', 'fontsize', 16, 'fontname', 'times');
legend('Ground Truth','Dead Reckoning','EKF SLAM','True LM','Estimated LM');
grid on;
axis equal;

figure(3);
%x1 = [result.time(:) result.error_lm(:)];
%set(gca, 'XTick', [0 30] );
set(gca, 'fontsize', 16, 'fontname', 'times');
plot(result.time(:),result.error_lm(:),'-r','linewidth',1); hold on;

title('Error on the estimates of landmark','fontsize', 16, 'fontname', 'times');
%xlim([0 30]);
xlabel('time (sec)','fontsize', 16, 'fontname', 'times');
ylabel('Error (m)', 'fontsize', 16, 'fontname', 'times');
grid on;
axis equal;

% figure(4);
% %set(gca, 'fontsize', 16, 'fontname', 'times');
% plot(result.time(:),result.uncertainty(:),'-r','linewidth',1); hold on;
% set(gca, 'XTick', [1:1:endtime]);
% title('Uncertainty regions of landmark','fontsize', 16, 'fontname', 'times');
% xlabel('time (sec)','fontsize', 16, 'fontname', 'times');
% ylabel('Uncertainty (m^2)', 'fontsize', 16, 'fontname', 'times');
% grid on;
% axis equal;
figure(5);
set(gca, 'fontsize', 16, 'fontname', 'times');
plot(result.time(:),toDegree(result.xTrue(:,3))-toDegree(result.xEst(:,3)),'-r','linewidth',1); hold on;
plot(result.time(:),toDegree(result.xTrue(:,3))-toDegree(result.xd(:,3)),'-g','linewidth',1); hold on;
%plot(result.time(:),abs(toDegree(result.xTrue(:,3))),'-k','linewidth',1)
title('Error on the estimates of robot pose','fontsize', 16, 'fontname', 'times');
xlabel('time (sec)','fontsize', 16, 'fontname', 'times');
ylabel('Error (m)', 'fontsize', 16, 'fontname', 'times');
legend('SLAM','Dead reckoning','fontsize');
grid on;
axis equal;


figure(6);
set(gca, 'fontsize', 16, 'fontname', 'times');
plot(result.time(:),result.error_robot(:),'-r','linewidth',1); hold on;
plot(result.time(:),result.error_robot_dead(:),'-g','linewidth',1); hold on;
title('Error on the estimates of robot position','fontsize', 16, 'fontname', 'times');
xlabel('time (sec)','fontsize', 16, 'fontname', 'times');
ylabel('Error (m)', 'fontsize', 16, 'fontname', 'times');
legend('SLAM','Dead reckoning','fontsize');
grid on;
axis equal;

% figure(7);
% for i=1:1:size(result.PEst,1)
% Px = [ result.PEst(i,1) result.PEst(i,2)];
% Py = [ result.PEst(i,3) result.PEst(i,4)];
% Pxy = [Px; Py];
%     [eigvec, eigval]=eig(Pxy);
%     
%     if eigval(1,1)>=eigval(2,2)
%     bigind=1;
%     smallind=2;
%     else
%     bigind=2;
%     smallind=1;
%     end
% 
% chi=9.21;
% %t=0:10:360;
% a=sqrt(eigval(bigind,bigind)*chi);
% b=sqrt(eigval(smallind,smallind)*chi);
% y = a*b*pi;
% plot(result.time(:), y,'-b','linewidth', 1);hold on;
% end
% title('Uncertainty regions of landmark','fontsize', 16, 'fontname', 'times');
% xlabel('Number of movements','fontsize', 16, 'fontname', 'times');
% ylabel('Error (m^2)', 'fontsize', 16, 'fontname', 'times');
% axis equal;
% grid on;




