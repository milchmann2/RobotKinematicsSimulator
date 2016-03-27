function draw_kin(koor,ks_length, p1, p2, p3, p1_2, p2_2, p3_2, p1_3, p2_3, p3_3)
%
% draws the base and tool frame of a robot
% and connects the origin of all frames with black lines
%
% Input parameters:
%   koor: cellarray containing all coordinate frames
%   ks_length: length for drawing frame axes
%
%************************************************
%   Prof.(FH) DI Dr. Robert Merz                *
%   2010                                        *
%                                               *
%   adapted from zeichne_kin_kette.m by         *
%   Prof. Georg Stark, Fachhochschule Augsburg  *
%************************************************

% koor
% ks_length
% assignin('base', 'koor', koor);
% pause
% calculate starting frame
[zz nk]=size(koor);

draw_frame(koor{1},ks_length);
p_vor=koor{1}(1:3,4);                         
for ii=2:nk
    p_neu= koor{ii}(1:3,4);   
    % draw line between frames
    plot3([p_vor(1) p_neu(1)],[p_vor(2) p_neu(2)],[p_vor(3) p_neu(3)],'Color','k','LineWidth',1);
    if ii==nk
         draw_frame(koor{ii},ks_length);
    end

    
    p_vor = p_neu;
end % for-loop
    plot3(p1(1, 4), p1(2, 4), p1(3, 4), 'bo');
    plot3(p2(1, 4), p2(2, 4), p2(3, 4), 'bo');
    plot3(p3(1, 4), p3(2, 4), p3(3, 4),'bo');
    
    a = p1_2;
    b = p2_2;
    c = p3_2;
%     plot(p1_2(1, 4), p1_2(2, 4), 'bo');
%     plot(p2_2(1, 4), p2_2(2, 4), 'bo');
%     plot(p3_2(1, 4), p3_2(2, 4), 'bo');

    plot3(p1_2(1, 4), p1_2(2, 4), p1_2(3, 4), 'bo');
    plot3(p2_2(1, 4), p2_2(2, 4), p2_2(3, 4), 'bo');
    plot3(p3_2(1, 4), p3_2(2, 4), p3_2(3, 4), 'bo');
%     p3_2(1, 4)
%     p3_2(2, 4)
%     p3_2(3, 4)
%     
    
    plot3(p1_3(1, 4), p1_3(2, 4), p1_3(3, 4), 'bo');
    plot3(p2_3(1, 4), p2_3(2, 4), p2_3(3, 4), 'bo');
    plot3(p3_3(1, 4), p3_3(2, 4), p3_3(3, 4), 'bo');

