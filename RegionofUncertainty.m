function [Area]=RegionofUncertainty(PEst)
Pxy = PEst(4:5,4:5);
    [eigvec, eigval]=eig(Pxy);
    
    if eigval(1,1)>=eigval(2,2)
    bigind=1;
    smallind=2;
    else
    bigind=2;
    smallind=1;
    end
    chi=9.21;
%t=0:10:360;
a=sqrt(eigval(bigind,bigind)*chi);
b=sqrt(eigval(smallind,smallind)*chi);
Area = a*b*pi;