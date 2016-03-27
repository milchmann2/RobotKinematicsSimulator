function draw_robot_path(q,t_ipo,robot,ks_length,erase, p1, p2, p3, p1_2, p2_2, p3_2, p1_3, p2_3, p3_3, gyroz, accelplot)
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

draw_kin(koor,ks_length, p1{1}, p2{1}, p3{1}, p1_2{1}, p2_2{1}, p3_2{1}, p1_3{1}, p2_3{1}, p3_3{1});

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

    draw_kin(koor,ks_length, p1{ii}, p2{ii}, p3{ii}, p1_2{ii}, p2_2{ii}, p3_2{ii}, p1_3{ii}, p2_3{ii}, p3_3{ii});
        
    % draw line between waypoints
    p_neu= koor{n+2}(1:3,4);
    %assignin('base', 'p_neu', p_neu);

    %pause
    plot3([p_vor(1) p_neu(1)],[p_vor(2) p_neu(2)],[p_vor(3) p_neu(3)],'Color','y','LineWidth',2);
    p_vor=p_neu;
        
    if accelplot == 1
        figure(2);
        legend('1', '2', '3', '4', 'Position',[60, 60, 60, 60]);
        plot(gyroz(1:ii,1), 'r')
        hold on;
        plot(gyroz(1:ii,2), 'ko')
        plot(gyroz(1:ii,3), 'bx')
        plot(gyroz(1:ii,4), 'g')
    end
%     valslength = size(gyroz(:,1), 1);
%     maxval = max(max(abs(gyroz)))*1.2;
%     axis([0 valslength -maxval maxval])
end % for-loop

