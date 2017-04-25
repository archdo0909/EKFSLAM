function [y,S,H]=CalcInno_Multicom(lm,xEst,PEst,z,LMID,LM_I)

global Q;

delta=lm-xEst(1:2);
q=delta'*delta;
zangle=atan2(delta(2),delta(2))-xEst(3);
i_p = LM_I * 0.0001 / (4 * pi * q);
zp = [i_p PI2PI(zangle)];
y=(z-zp)';
H=jacobH(q,delta,xEst,LMID);
S=H*PEst*H'+Q;
end 