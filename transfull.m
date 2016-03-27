function [ T ] = transfull( T, R )
%TRANSFULL Summary of this function goes here
%   Detailed explanation goes here
    T = R+T;
    T(4, 4) = 1;
end

