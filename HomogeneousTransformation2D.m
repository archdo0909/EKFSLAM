function out = HomogeneousTransformation2D(in, base, mode)

Rot=[cos(base(3)) sin(base(3)); -sin(base(3)) cos(base(3))];

Nin=size(in);
baseMat=repmat(base(1:2),Nin(1),1);


if Nin(2)>=3
    inxy=in(:,1:2);
    inOther=in(:,3:end);
    in=inxy;
end

%????
if nargin==2 || mode==0 %?????
    out=baseMat+in*Rot;
else %?????
    out=(baseMat+in)*Rot;
end
    
%?????????????
if Nin(2)>=3
    out=[out inOther];
end
end
