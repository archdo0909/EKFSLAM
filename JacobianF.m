% close all;
% clear all;
% clc;
% 
% syms x y theta sl sr B
% u = [sr sl]';
% X = [x y theta]';
% d_r = (u(1)-u(2))/(2*B);
% d_t = (u(1)+u(2))/2;
% 
% F = [ 1 0 -d_t*sin(X(3) + d_r);   
%       0 1 d_t*cos(X(3) + d_r);
%       0 0 1];
function [G, Fx]=JacobianF(x, u)

global dt;
global PoseSize;
global LMSize;

Fx = horzcat(eye(PoseSize), zeros(PoseSize,LMSize*GetnLM(x)));

jF = [0 0 -dt*u(1)*sin(x(3));
      0 0 dt*u(1)*cos(x(3));
      0 0 0];
 
G=eye(length(x))+Fx'*jF*Fx;   %(3x3)'


end