function [z,x,xd,u]=Observation_com(x,xd,u,LM,I)
    
    global Qsigma;
    %global Wsigma;
    x = motion(x, u);
    u = u+Qsigma*randn(2,1);
    xd = motion(xd,u); %Dead reckoning
    z=[];
    for iz=1:length(LM(:,1))
      yaw = zeros(3,1);
      yaw(3)=-x(3);
      localLM = HomogeneousTransformation2D(LM(1:2)'-x(1:2)',yaw');
      d = norm(localLM);
      % 8cm * 8cm = 0.08 * 0.08 = 0.08 * 0.08 m^2 = 0.0064
      % 4 * pi * d ^ 2
      i = I * 0.0064 / (4 * pi * d^2);
      %z = PI2PI(atan2(localLM(2),localLM(1)));
      if i < 1000
        max_t=around(1000/i);
          
          
      end
    end
      
      
end