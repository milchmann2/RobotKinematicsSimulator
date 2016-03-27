function T = xyzabc_2_t(x,y,z,a,b,c)
    %xyzabc_2_t	Converts x,y,z, a,b,c into transformatin matrix
    %
    %	T = xyzabc_2_t(x,y,z,a,b,c)  calculates the transformation matrix
    %	from the displacement and the roll, pitch and yaw angles
    %	
    %   x ... x coordinate of displacement
    %	y ... y coordinate of displacement
    %	z ... z coordinate of displacement
    %	a ... yaw angle (euler angle around Z)
    %	b ... pitch angle (euler angle around Y)
    %   c ... roll angle (euler angle around X)
    %	
    %   ALL ANGELS IN DEG!!
    error(nargchk(6,6,nargin));
    %****************************************************************
    %Implement your code here
    T = [cosd(a)*cosd(b) cosd(a)*sind(b)*sind(c)-sind(a)*cosd(c) cosd(a)*sind(b)*cosd(c)+sind(a)*sind(c) x;
        sind(a)*cosd(b) sind(a)*sind(b)*sind(c)+cosd(a)*cosd(c) sind(a)*sind(b)*cosd(c)-cosd(a)*sind(c) y;
        -sind(b) cosd(b)*sind(c) cosd(b)*cosd(c) z;
        0 0 0 1];
end
        
        