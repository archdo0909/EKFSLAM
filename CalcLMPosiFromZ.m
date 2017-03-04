function zl=CalcLMPosiFromZ(x,z,len)
%     zl=x(1:2)+[z(1)*cos(x(3)+z(2));
%                z(1)*sin(x(3)+z(2))];
    
    zl = x(1:2) + [len*cos(x(3)+z(2));
                   len*sin(x(3)+z(2))];
end