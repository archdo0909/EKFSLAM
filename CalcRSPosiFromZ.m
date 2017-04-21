function zl=CalcRSPosiFromZ(x,z,I)
%Calculate Radiation Sources from Observation data z
	%z(1) is a intensity which gets inside to camera 
 
	L = sqrt(I/(4*pi*z(1)));

    zl=x(1:2) + [L*cos(x(3)+z(2)); L*sin(x(3)+z(2))];


end