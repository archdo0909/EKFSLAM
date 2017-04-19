function [z]=Observation_MultiCom(x,u,LM,I,time)
      
%     global Qsigma;
%     %global Threshold_Dis
%     %global Wsigma;
      x = motion(x, u);
%     u = u+Qsigma*randn(2,1);
%     xd = motion(xd,u); %Dead reckoning
      z=[];
%     %t_max = -100;
      
    for iz=1:length(LM(:,1))
      yaw = zeros(3,1);
      yaw(3)=-x(3);
      %localLM = HomogeneousTransformation2D(LM(1:2)'-x(1:2)',yaw');
      localLM = HomogeneousTransformation2D(LM(iz,:)'-x(1:2)',yaw');
      d = norm(localLM);
      
      % 8cm * 8cm = 0.08 * 0.08 = 0.08 * 0.08 m^2 = 0.0064
      % 4 * pi * d ^ 24
      
      i = I * 0.0064 / (4 *pi * d^2) * 3; %%%cosin(theta) nowhere
      
      z = [z; [i PI2PI(atan2(localLM(2), localLM(1))) LM(iz,:) time]];
      
    end
      
end   