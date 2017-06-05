function [PAug, xAug]=Augmented_data_Multibearing(z,xEst,PEst,LM_I,len)
    
    global Q
    global Ssigma
    zl = xEst(1:2) + [len*cos(xEst(3) + z);
                      len*sin(xEst(3) + z)];
                  
    
   Yx = [1  0  -len*sin(z + xEst(3));
         0  1  len*cos(z + xEst(3))];
  
    Yz = [ -len*sin(z+xEst(3));
           len*cos(z+xEst(3))];
    Ys = [cos(z+xEst(3));
          sin(z+xEst(3))]; 

    Px_y = PEst(1:3,1:3);  
      
    if(GetnLM(xEst) >= 1)
        for i=1:GetnLM(xEst)
            Px_y = [Px_y PEst(1:3,4+2*(i-1):5+2*(i-1))]; %(3 * 3 or 5 or 7...)
        end 
    end
    Px = Yx*Px_y;  % ( 2 * 3 or 5 or 7... )y  
    
    Ppp = Yx*PEst(1:3,1:3)*Yx' + Yz*Q*Yz' + Ys*Ssigma^2*Ys';  %(2 X 2)
    
    PAug = [PEst Px';
            Px  Ppp];
    xAug=[xEst; zl];
    
    xEst = xAug;
    PEst = PAug;

end