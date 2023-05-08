clc;clear all;close all;
%% Making the lookup table.
% Just saving the value from A* heuristics. 
n = 100;
[x,y] = meshgrid(1:.1:5,1:.1:5);
r = sqrt(x^2+y^2);
tha= atan(y/x);
T_tha = [cos(tha),-sin(tha),0;sin(tha) cos(tha) 0;0,0,1];
kr = 1;kz = 1;
z = 1;
g = @(r,tha,z) 1/sqrt(kr^2*(r-1)^2+1+kz^2*(z-1)^2) * T_tha *[-kr*(r-1),1,-kz*(z-1)]';