function zl=CalcRSPosiFromZ(xEst,z,LM_I)
%Calculate Radiation Sources from Observation data z
	%z(1) is a intensity which gets inside to camera 
 
% 	 L = sqrt(LM_I *10^(-4)*100 /(4*pi*z(1)));
% 
%     zl=xEst(1:2) + [L*cos(xEst(3)+z(2)); L*sin(xEst(3)+z(2))];
     zl = xEst(1:2)+[z(1)*cos(xEst(3)+z(2)); z(1)*sin(xEst(3)+z(2))];

end