function T = rotx(a)
% Returns the transformation matrix for a rotation around the x-axis
% a: angle in deg
% T: homogenious 4*4-matrix

%   ALL ANGLES in DEG!!

error(nargchk(1,1,nargin));

%****************************************************************
%Implement your code here

T=[1 0 0 0; 0 cosd(a) -sind(a) 0; 0 sind(a) cosd(a) 0; 0 0 0 1];