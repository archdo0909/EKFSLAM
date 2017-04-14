function [z,x,IN,xd,u,t_max]=Observation_com(x,xd,u,LM,I)
    
    global Qsigma;
    %global Wsigma;
    x = motion(x, u);
    u = u+Qsigma*randn(2,1);
    xd = motion(xd,u); %Dead reckoning
    z=[];
    IN=[];
    t_max = -100;
    
    for iz=1:length(LM(:,1))
      yaw = zeros(3,1);
      yaw(3)=-x(3);
      %localLM = HomogeneousTransformation2D(LM(1:2)'-x(1:2)',yaw');
      localLM = HomogeneousTransformation2D(LM(iz,:)'-x(1:2)',yaw');
      d = norm(localLM);
      % 8cm * 8cm = 0.08 * 0.08 = 0.08 * 0.08 m^2 = 0.0064
      % 4 * pi * d ^ 24
      
      i = I * 0.0064 / (4 *pi * d^2); %%%cosin(theta) nowhere
      %i = 50;
      %z = PI2PI(atan2(localLM(2),localLM(1)));
      if i < 1000
        t = ceil(1000 / i);
      else
        t = 1;
      end
      
      if t_max < t
          t_max = t;
      end
      IN = [IN; i];
      z = [z; PI2PI(atan2(localLM(2), localLM(1)))];
    
    end
    
end