function draw_robot_path(q,t_ipo,robot,ks_length,erase, p)
%
% draws the path of the robot tool 
%
% Input parameters:
%   q ...... array of column vectors of joint variables
%   t_ipo ... interpolation clock
%   robot: robot parameters
%   ks_length: length for drawing frame axes
%   erase: display will be cleared if =1
%
%************************************************
%   Prof.(FH) DI Dr. Robert Merz                *
%   (c) 2010                                    *
%************************************************


%error(nargchk(5,5,nargin));

% calculate starting frame
[n nq]=size(q);

koor=coortraf_craig(q(:,1),robot);
%draw_frame(koor{n+2},ks_length);

if erase==1 
    cla;
end


draw_kin(koor,ks_length, p(1,:));

p_vor=koor{n+2}(1:3,4);
pause

for ii=2:nq

    pause(t_ipo);
    figure(1)
%     pause

    koor=coortraf_craig(q(:,ii),robot);
    %draw_frame(koor{n+2},ks_length);
    if erase==1 
        cla;
    end

    draw_kin(koor,ks_length, p(ii,:));
        
    % draw line between waypoints
    p_neu= koor{n+2}(1:3,4);
    %assignin('base', 'p_neu', p_neu);

    %pause
    plot3([p_vor(1) p_neu(1)],[p_vor(2) p_neu(2)],[p_vor(3) p_neu(3)],'Color','y','LineWidth',2);
    p_vor=p_neu;
        
end 

