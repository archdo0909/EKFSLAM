function Animation_test(result,LM)

figure(1);
hold off;
plot(result.xTrue(:,1), result.xTrue(:,2),'.b');hold on;
plot(LM(:,1),LM(:,2),'pk','MarkerSize',10);hold on;
axis equal;
grid on;

drawnow;