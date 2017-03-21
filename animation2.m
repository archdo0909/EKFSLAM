function animation2(PEst,time)
figure(123);
Pxy = PEst(4:5,4:5);
    [eigvec, eigval]=eig(Pxy);
    
    if eigval(1,1)>=eigval(2,2)
    bigind=1;
    smallind=2;
    else
    bigind=2;
    smallind=1;
    end

chi=9.21;
%t=0:10:360;
a=sqrt(eigval(bigind,bigind)*chi);
b=sqrt(eigval(smallind,smallind)*chi);
y = a*b*pi;
plot(time, y,'.r');hold on;
%title('Uncertainty regions of landmark','fontsize', 16, 'fontname', 'times');
%xlabel('Number of movements','fontsize', 16, 'fontname', 'times');
%ylabel('Error (m^2)', 'fontsize', 16, 'fontname', 'times');
axis equal;
grid on;
drawnow;

end