function [PAug, xAug]=Augmented_data_Multicom(z,xEst,PEst,LM_I)
    
    global Q
    global Ssigma
    
    L = sqrt(LM_I/(4*pi*z(1)));

    zl=xEst(1:2) + [L*cos(xEst(3)+z(2)); L*sin(xEst(3)+z(2))];
  
    
    Yx = [1 0 -L*sin(xEst(3)+z(2));
          0 1  L*cos(xEst(3)+z(2))];
    
    Yz = [-L*sin(z(2)+xEst(3));
           L*cos(z(2)+xEst(3))];
    Ys = [cos(z(2)+xEst(3));
          sin(z(2)+xEst(3))];
    
    Px_y = PEst(1:3,1:3);  
      
    if(GetnLM(xEst) >= 1)
        for i=1:GetnLM(xEst)
            Px_y = [Px_y PEst(1:3,4+2*(i-1):5+2*(i-1))]; %(3 * 3 or 5 or 7...)
        end 
    end
    Px = Yx*Px_y;  % ( 2 * 3 or 5 or 7... )
    Ppp = Yx*PEst(1:3,1:3)*Yx' + Yz*Q*Yz' + Ys*Ssigma^2*Ys';
    
    %PAug_1 = horzcat(PEst, Px');
    %PAug_2 = horzcat(Px,Ppp);
    %PAug = vertcat(PAug_1, PAug_2);
    PAug = [PEst Px';
            Px Ppp];    
    xAug = [xEst; zl];    
    
end