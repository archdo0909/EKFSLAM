function [PEst, xEst]=Data_Augmentation(z,xEst,PEst,len)
    
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



%         zl = xEst(1:2) + [len*cos(xEst(3) + z(2));
%                       len*sin(xEst(3) + z(2))];
%                   
%     
%    Yx = [1  0  -len*sin(z(2) + xEst(3));
%          0  1  len*cos(z(2) + xEst(3))];
%   
%     Yz = [-len*sin(z(2)+xEst(3))
%            len*cos(z(2)+xEst(3))];
%     Ys = [cos(z(2)+xEst(3));
%           sin(z(2)+xEst(3))];  

    Px = Yx*PEst;  %(2 X 3)
    Ppp = Yx*PEst*Yx' + Yz*Q*Yz' + Ys*Ssigma^2*Ys';  %(2 X 2)
    
    PAug = [PEst Px';
            Px  Ppp];
    xAug=[xEst; zl];
    
    xEst = xAug;
    PEst = PAug;
    
end