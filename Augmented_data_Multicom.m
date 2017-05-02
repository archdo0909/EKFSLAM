function [PAug, xAug]=Augmented_data_Multicom(z,xEst,PEst,LM_I)
    
    global Q
    global Ssigma
    %initP=eye(2)*1000;
    L = sqrt(LM_I *10^(-4)*100 /(4*pi*z(1)));

    zl=xEst(1:2) + [L*cos(xEst(3)+z(2)); L*sin(xEst(3)+z(2))];
    %zl = xEst(1:2)+[z(1)*cos(xEst(3)+z(2)); z(1)*sin(xEst(3)+z(2))];
    
    Yx = [1 0 -L*sin(xEst(3)+z(2));
          0 1  L*cos(xEst(3)+z(2))];
    
%     Ys = [-L*sin(z(2)+xEst(3));
%            L*cos(z(2)+xEst(3))];
%     Yz = [cos(z(2)+xEst(3));
%           sin(z(2)+xEst(3))];   
    Yz = [cos(z(2)+xEst(3)) -L*sin(z(2)+xEst(3));
          sin(z(2)+xEst(3)) L*cos(z(2)+xEst(3))];   

%     Ys = [cos(z(2)+xEst(3));
%           sin(z(2)+xEst(3))];
    
%     Yx = [1 0 -z(1)*sin(xEst(3)+z(2));
%           0 1 z(1)*cos(xEst(3)+z(2))];
%     Yz = [cos(z(2)+xEst(3))  -z(1)*sin(z(2)+xEst(3));
%           sin(z(2)+xEst(3))  z(1)*cos(z(2)+xEst(3))];
       
    Px_y = PEst(1:3,1:3);  
      
    if(GetnLM(xEst) >= 1)
        for i=1:GetnLM(xEst)
            Px_y = [Px_y PEst(1:3,4+2*(i-1):5+2*(i-1))]; %(3 * 3 or 5 or 7...)
        end 
    end
   Px = Yx*Px_y;  % ( 2 * 3 or 5 or 7... )y
   
    %Ppp = Yx*PEst(1:3,1:3)*Yx' + Yz*Q*Yz' + Ys*Ssigma^2*Ys';
    Ppp = Yx*PEst(1:3,1:3)*Yx'+Yz*Q*Yz';

    PAug = [PEst Px';
            Px Ppp];    
    xAug = [xEst; zl];    
%     

%     xAug=[xEst;zl];
%     PAug=[PEst zeros(length(xEst),LMSize);
%           zeros(LMSize,length(xEst)) initP];
end