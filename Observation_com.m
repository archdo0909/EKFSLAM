function [z,x,xd,u]=Observation_com(x,xd,u,LM)
    
    global Qsigma;
    global Rsigma;
    x = motion(x, u);
    u = u+Qsigma*randn(2,1);
    xd = motion(xd,u); %Dead reckoning
    %z=[];
    for iz = 1:length(LM(:,1))
      yaw = zeros(3,1);
      yaw(3)=-x(3);
      localLM = HomogeneousTransformation2D(LM(iz,:)-x(1:2)',yaw');
      %localLM = HomogeneousTransformation2D(LM(iz,:)'-x(1:2)',yaw');
      %d = norm(localLM);
      noise =Rsigma*randn(2,1);
      % 8cm * 8cm = 0.08 * 0.08 = 0.08 * 0.08 m^2 = 0.0064
      % 4 * pi * d ^ 24
      
      z = [z; PI2PI(atan2(localLM(2), localLM(1))+noise(2)) LM(iz,:)];
    end
    
end