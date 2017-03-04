function angle=PI2PI(angle)

    angle = mod(angle, 2*pi);
    
    i = find(angle>pi);
    angle(i)=angle(i)-2*pi;
    
    i = find(angle<-pi);
    angle(i)=angle(i) + 2*pi;


end