classdef Gyros
    %GYROS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        GyroX
        GyroY
        GyroZ 
        pp1 = [1 0 0 0; 0 1 0 0; 0 0 1 0; 1, 1, 0,1 ]';
        pp2 = [1 0 0 0; 0 1 0 0; 0 0 1 0; 1, -1, 0, 1]';
        pp3 = [1 0 0 0; 0 1 0 0; 0 0 1 0; -1, 0, 0, 1]';
%         pp1 = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0, 1, 1,1 ]';
%         pp2 = [1 0 0 0; 0 1 0 0; 0 0 1 0; -1, -1, 2, 1]';
%         pp3 = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0, -1, 0, 1]';
    end
    
    methods
        function [rot, tr] = CalculateRotations(obj, q, px, py, pz, robot)
            % cc2
            if (size(robot.Joints.AnglesX,2) == 1)
                 for i=1:size(q,1);
                % cc2 before i had only rotx(robot.Joints.AnglesX(i))
%                 x = rotx(robot.Joints.AnglesX(i));
%                 y = roty(robot.Joints.AnglesY(i));
%                 z = rotz(robot.Joints.AnglesZ(i));
                x = rotx(q(1));
                y = roty(q(2));
                z = rotz(q(3));
                rot{i} = z*y*x;
%                     rot{i}{1} = 0;
                 end
            else
                for i=1:size(q,1);
                    % cc2 before i had only rotx(robot.Joints.AnglesX(i))
%                     x = rotx(robot.Joints.AnglesX(i)-robot.Joints.AnglesX(i-1));
%                     y = roty(robot.Joints.AnglesY(i)-robot.Joints.AnglesY(i-1));
%                     z = rotz(robot.Joints.AnglesZ(i)-robot.Joints.AnglesZ(i-1));
                    x = rotx(q(1));
                    y = roty(q(2));
                    z = rotz(q(3));
                    rot{i} = z*y*x;
    %                     rot{i}{1} = 0;
                end
            end
%             rot{1} = rotz(q(1));
%             rot{2} = rotz(q(2));
%             rot{3} = rotz(q(3));
            
            tr{1} = trans(px(2), py(2), pz(2));
            tr{2} = trans(px(3), py(3), pz(3));
            tr{3} = trans(px(4), py(4), pz(4));
        end
        
        function p = CalculatePosition(obj, tr)  
            p{1} = tr*obj.pp1;
            p{2} = tr*obj.pp2;
            p{3} = tr*obj.pp3;
        end
            
        function pr = CalculateR(obj, rot, p)          
%             p{1} = tr1*obj.pp1;
%             p{2} = tr1*obj.pp2;
%             p{3} = tr1*obj.pp3;  
            if (~iscell(p))
                pr{1} = rot*obj.pp1;
                pr{2} = rot*obj.pp2;
                pr{3} = rot*obj.pp3;
            else
                pr{1} = rot*p{1};
                pr{2} = rot*p{2};
                pr{3} = rot*p{3};
            end
        end
        
        function pt = CalculateT(obj, rot, p, t)
            if (~iscell(p))
                pt{1} = rot*obj.pp1 + t;
                pt{2} = rot*obj.pp2 + t;
                pt{3} = rot*obj.pp3 + t;
            else
                pt{1} = rot*p{1} + t;
                pt{2} = rot*p{2} + t;
                pt{3} = rot*p{3} + t;
            end
        end
    end
    
end

