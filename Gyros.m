classdef Gyros
    %GYROS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        GyroX
        GyroY
        GyroZ 
        pp = cell(1,1)
%                 pp1 = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0, 1, 1,1 ]';
%         pp2 = [1 0 0 0; 0 1 0 0; 0 0 1 0; -1, -1, 2, 1]';
%         pp3 = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0, -1, 0, 1]';
    end
    
    methods
        function obj = Gyros()
            % pp{1} = x
            % pp{2} = y
            % pp{3} = z
            obj.pp{1}{1} = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0, -1, 1, 1]';
            obj.pp{1}{2} = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0, 1, 1, 1]';
            obj.pp{1}{3} = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0, 0, -1, 1]';

            obj.pp{2}{1} = [1 0 0 0; 0 1 0 0; 0 0 1 0; 1, 0, 1, 1]';
            obj.pp{2}{2} = [1 0 0 0; 0 1 0 0; 0 0 1 0; -1, 0, 1, 1]';
            obj.pp{2}{3} = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0, 0, -1, 1]';

            obj.pp{3}{1} = [1 0 0 0; 0 1 0 0; 0 0 1 0; 1, 1, 0,1]';
            obj.pp{3}{2} = [1 0 0 0; 0 1 0 0; 0 0 1 0; 1, -1, 0, 1]';
            obj.pp{3}{3} = [1 0 0 0; 0 1 0 0; 0 0 1 0; -1, 0, 0, 1]';
        
        end
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
            solution = t_2_xyzabc(tr, 1);
            angle(1) = solution(3);
            angle(2) = solution(2);
            angle(3) = solution(1);
            T_t = trans(solution(4), solution(5), solution(6));
            T_t = [0 0 0 solution(4); 0 0 0 solution(5); 0 0 0 solution(6); 0 0 0 0];
            T{1} = rotx(solution(3)) + T_t;
            T{2} = roty(solution(2)) + T_t;
            T{3} = rotz(solution(1)) + T_t;
            for i=1:3
                for j=1:3
                    p{i}{j} = T{i}*obj.pp{i}{j};
%                     p{i}{j} = T{j}*obj.pp{i}{j};
%                     p{i}{j} = T{j}*obj.pp{i}{j};
                end
            end
        end
            
        function pr = CalculateR(obj, rot, p)          
%             p{1} = tr1*obj.pp1;
%             p{2} = tr1*obj.pp2;
%             p{3} = tr1*obj.pp3;  
            if (~iscell(p))
                for i=1:3
                    for j=1:3
                        pr{i, j} = rot*obj.pp{i}{j};
                        pr{i, j} = rot*obj.pp{i}{j};
                        pr{i, j} = rot*obj.pp{i}{j};
                    end 
                end
            else
                for i=1:3
                    for j=1:3                
                        pr{i, j} = rot*p{i, j};
                        pr{i, j} = rot*p{i, j};
                        pr{i, j} = rot*p{i, j};
                    end
                end
                
            end
        end
    end
    
end

