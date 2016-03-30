function solution = t_2_xyzabc(T,pose)
    %t_2_xyzabc	Converts transformation matrix into x,y,z, a,b,c  
    %
    %	[x,y,z,a,b,c] = t_2_xyzabc(T)  
    %	calculates the displacement and the 
    %	roll, pitch and yaw angles from the transformation matrix
    %	
    %	pose = 1 or 2 - determines solution
    %
    %  	x ... x coordinate of displacement
    %	y ... y coordinate of displacement
    %	z ... z coordinate of displacement
    %	a ... yaw angle (euler angle around Z)
    %	b ... pitch angle (euler angle around Y)
    %  	c ... roll angle (euler angle around X)
    %	
    %   ALL ANGELS in DEG!!


    error(nargchk(2,2,nargin));

    [m,n] = size(T);
    if ((n ~= 4)|(m ~= 4))
    	error('Transformation Matrix must be 4 by 4');
    end

    %****************************************************************
    %Implement your code here
    
%     if ((abs(T(1,3)) < 1e-7) && (abs(T(1,3)) < 1e-7))  %abs(T(1,3)) < 1e-12*ones(size(A)
%         b = 0;
%         a = 0;
%         c = 0;
%     else
%         b = atan2d(T(3,3), sqrt(1-T(3,3)^2));
%         % ^can also be minus here!
%         % v also changes the formulas below
%         a = atan2d(T(1,3), T(2,3));
%         c = atan2d(-T(3, 1), T(3,2));
%     end
    
    
%     if pose == 1
%         b = rad2deg(atan2(-T(3,1),+sqrt(T(1,1)^2+T(2,1)^2)));
%     elseif pose == 2
%         b = rad2deg(atan2(-T(3,1),-sqrt(T(1,1)^2+T(2,1)^2)));
%     end
%             
%     a = rad2deg(atan2(T(2,1)/cos(b), T(1,1)/cos(b)));
%     c = rad2deg(atan2(T(3,2)/cos(b), T(3,3)/cos(b)));
%     x = T(1,4);
%     y = T(2,4);
%     z = T(3,4);



    b = rad2deg(atan2(-T(3,1), sqrt(T(1,1)^2 + T(2,1)^2)));
    
    if b == 90
        a = 0;
        c = rad2deg(atan2(T(1,2), T(2,2)));
    elseif b == -90
        a = 0;
        c = rad2deg(-atan2(T(1,2), T(2,2)));
    else
        a = rad2deg(atan2(T(2,1)/cosd(b), T(1,1)/cosd(b)));
        c = rad2deg(atan2(T(3,2)/cosd(b), T(3,3)/cosd(b)));
    end
    
    x = T(1,4);
    y = T(2,4);
    z = T(3,4);
    
    solution = [a,b,c,x,y,z];

end

