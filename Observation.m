function [z, x, xd, u]=Observation(x, xd, u, LM, MAX_RANGE)

    global Qsigma;
    global Rsigma;
    
    x = motion(x, u);%Ground Truth
    u=u+Qsigma*randn(2,1);%add Process Noise
    xd=motion(xd, u);%Dead Reckoning
    
    %Simulate Observation
    z=[];
    for iz=1:length(LM(:,1))
        yaw = zeros(3,1);
        yaw(3)=-x(3);
        localLM=HomogeneousTransformation2D(LM(iz,:)-x(1:2)',yaw');
        d=norm(localLM);
        
        if d<MAX_RANGE %Observation range
            noise=Rsigma*randn(2,1);
            z=[z; [d+noise(1) PI2PI(atan2(localLM(2),localLM(1)) + noise(2)) LM(iz,:)]];
        end
    end
    
end 