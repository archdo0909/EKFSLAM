function [z]=Observation_multi_bearing(x,u,LM)
    
   
    global Rsigma;
   
    z=[];
    for iz = 1:length(LM(:,1))
      yaw = zeros(3,1);
      yaw(3)=-x(3);
      localLM = HomogeneousTransformation2D(LM(iz,:)-x(1:2)',yaw');
      noise =Rsigma*randn(2,1);
      
      z = [z; PI2PI(atan2(localLM(2), localLM(1))+noise(2)) LM(iz,:)];
    end
    
end