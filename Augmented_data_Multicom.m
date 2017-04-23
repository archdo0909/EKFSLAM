function [PEst, xEst]=Augmented_data_Multicom(z,xEst,PEst,LM_I)
    
    global Q
    global Ssigma
    
    L = sqrt(LM_I/(4*pi*z(1)));

    zl=x(1:2) + [L*cos(xEst(3)+z(2)); L*sin(xEst(3)+z(2))];
    
    Yx = [1 0 -L*sin(xEst(3)+z(2));
          0 1  L*cos(xEst(3)+z(2))];
    
    Yz = [-L*sin(z(2)+xEst(3));
           L*cos(z(2)+xEst(3))];
    Ys = [cos(z(2)+xEst(3));
          sin(z(2)+xEst(3))];
      
    Px = Yx * PEst;
    Ppp = Yx*PEst*Yx' + Yz*Q*Yz' + Ys*Ssigma^2*Ys';
    
    PAug = [PEst Px';
            Px Ppp];
    xAug = [xEst; zl];    
    
    xEst = xAug;
    PEst = PAug;

end