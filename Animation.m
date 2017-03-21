function Animation(result,xTrue,LM,z,xEst,PEst)
figure(1);
hold off;
plot(result.xTrue(:,1),result.xTrue(:,2),'.b');hold on;
plot(LM(1,1),LM(2,1),'pk','MarkerSize',10);hold on;
% if~isempty(z)
%     for iz=1:length(z(:,1))
%         ray=[xTrue(1:2)';z(iz,3:4)];                %%% [ x  y ]
%         plot(ray(:,1),ray(:,2),'-r');hold on;       %%% [ z1 z2]
%     end
% end
%SLAM??????
% for il=1:GetnLM(xEst)
%     plot(xEst(4+2*(il-1)),xEst(5+2*(il-1)),'.c');hold on;
% end
plot(xEst(4),xEst(5),'diamond','MarkerSize',10);hold on;
%plot(zl(1,:),zl(2,:),'.b');hold on;
%ShowErrorEllipse(xEst,PEst);
ShowErrorEllipse_LM(xEst,PEst);
plot(result.xd(:,1),result.xd(:,2),'.k');hold on;
plot(result.xEst(:,1),result.xEst(:,2),'.r');hold on;
arrow=0.5;
x=result.xEst(end,:);
quiver(x(1),x(2),arrow*cos(x(3)),arrow*sin(x(3)),'ok');hold on;
axis equal;
grid on;
%?????????
%movcount=movcount+1;
%mov(movcount) = getframe(gcf);% ??????????????????
drawnow;
