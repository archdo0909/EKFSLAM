function DrawGraph_test(result,LM)

% figure(2);
% hold off;
% x=[result.xTrue(:,1:2) result.xEst(:,1:2)];
% set(gca, 'fontsize', 16, 'fontname', 'times');
% plot(x(:,1), x(:,2),'-b','linewidth', 2); hold on;
% plot(x(:,3), x(:,4),'-r','linewidth', 2); hold on;
% plot(LM(1),LM(2),'pk','MarkerSize',10);hold on;
% 
% % for il=1:GetnLM(xEst)
% %     plot(xEst(4+2*(il-1)),xEst(5+2*(il-1)),'diamond','MarkerSize',10);hold on;
% %     %plot(xEst(4+2*(il-1)),xEst(5+2*(il-1)),'.g');hold on;
% % end
% title('EKF SLAM Result', 'fontsize', 16, 'fontname', 'times');
% xlabel('X (m)', 'fontsize', 16, 'fontname', 'times');
% ylabel('Y (m)', 'fontsize', 16, 'fontname', 'times');
% legend('Ground Truth','Dead Reckoning','EKF SLAM','True LM','Estimated LM');
% grid on;
% axis equal;
% 
% 
% figure(3);
% set(gca, 'fontsize', 16, 'fontname', 'times');
% %plot(result.xEst(:,4),result.xEst(:,1),'-r','linewidth',1); hold on;
% %plot(result.xEst(:,4),result.xEst(:,2),'-b','linewidth',1); hold on;
% plot(result.u(:,2),result.u(:,3),'-b','linewidth',1); hold on;
% title('Error on the estimates of landmark','fontsize', 16, 'fontname', 'times');
% %xlim([0 30]);
% xlabel('time (sec)','fontsize', 16, 'fontname', 'times');
% ylabel('Error (m)', 'fontsize', 16, 'fontname', 'times');
% grid on;
% axis equal;

figure(1);
hold off;
plot(result.xTrue(:,1), result.xTrue(:,2),'-b','linewidth', 2); hold on;
title('EKF SLAM Result', 'fontsize', 16, 'fontname', 'times');
xlabel('X (m)', 'fontsize', 16, 'fontname', 'times');
ylabel('Y (m)', 'fontsize', 16, 'fontname', 'times');
legend('Ground Truth','Dead Reckoning','EKF SLAM','True LM','Estimated LM');
grid on;
axis equal;

end