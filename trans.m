function T = trans(x,y,z)
% Returns the transformation matrix for a translation
% x,y,z: coordinates of translational vector
% T: homogenious 4*4-Matrix

error(nargchk(3,3,nargin));

%****************************************************************
%Implement your code here

T=[1 0 0 x; 0 1 0 y; 0 0 1 z; 0 0 0 1];