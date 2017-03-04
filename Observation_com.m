function [z,x,xd,u]=Observation_com(x,xd,u,LM,len)
    
    global Qsigma;
    %global Wsigma;
    x = motion(x, u);
    u = u+Qsigma*randn(2,1);
    xd = motion(xd,u); %Dead reckoning
    %z=[];
      yaw = zeros(3,1);
      yaw(3)=-x(3);
      localLM = HomogeneousTransformation2D(LM(1:2)'-x(1:2)',yaw');
      %d=norm(localLM);
      %z = [len PI2PI(atan2(localLM(2),localLM(1)))]';
       z = PI2PI(atan2(localLM(2),localLM(1)));
%     z_true =  [len PI2PI(atan2( (x(2) - LM(2)),(x(1)-LM(1)) )) - x(3)]';
%     z=[len PI2PI(atan2( (xd(2) - LM(2)),(xd(1)-LM(1)) )) - xd(3)]';
    
    
end