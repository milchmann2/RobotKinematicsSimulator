classdef Gyro < handle
    %GYROS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        ValuesX;
        ValuesY;
        ValuesZ;
        Positions;
        NumberOfGyros;
        OmegasX;
        OmegasY;
        OmegasZ;
        pp = cell(1,1)
%                 pp1 = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0, 1, 1,1 ]';
%         pp2 = [1 0 0 0; 0 1 0 0; 0 0 1 0; -1, -1, 2, 1]';
%         pp3 = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0, -1, 0, 1]';
    end
    
    methods
        function obj = Gyro(numberOfJoints)
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
            obj.NumberOfGyros = numberOfJoints;
            obj.OmegasX = 0;
            obj.OmegasY = 0;
            obj.OmegasZ = 0;
        end
        
        
        function [rot, tr] = CalculateRotations(obj, q, px, py, pz, robot)
            % cc2
            if (size(robot.Joints.AnglesX,2) == 1)
                 for i=1:size(q,1);
                x = rotx(q(1));
                y = roty(q(2));
                z = rotz(q(3));
                rot{i} = z*y*x;
                 end
            else
                for i=1:size(q,1);
                    x = rotx(q(1));
                    y = roty(q(2));
                    z = rotz(q(3));
                    rot{i} = z*y*x;
                end
            end 
            tr{1} = trans(px(2), py(2), pz(2));
            tr{2} = trans(px(3), py(3), pz(3));
            tr{3} = trans(px(4), py(4), pz(4));
        end
        
        function p = CalculatePosition(obj, tr, opt, opt2)
            if nargin == 2
                opt = 0;
                opt2 = 0;
            end
            
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
                    if opt2 == 6 && opt > 1 && i == 1
                        p{i}{j} = T{i}*obj.pp{i}{j};
                    else
                        p{i}{j} = T{i}*obj.pp{i}{j};
                    end
                end
            end
        end
            
        
        function CalculateAngularVelocities(obj, timeStep)
            idx = size(obj.OmegasX, 1) + 1;
            for i=1:obj.NumberOfGyros
                for j=1:3   %x,y,z
                    cA = (obj.Positions{end-1, i}{j}{1}+obj.Positions{end-1, i}{j}{2}+obj.Positions{end-1, i}{j}{3})/3;
                    cB  = (obj.Positions{end, i}{j}{1}+obj.Positions{end, i}{j}{2}+obj.Positions{end, i}{j}{3})/3;            
                    H = ((obj.Positions{end-1, i}{j}{1}-cA)*(obj.Positions{end, i}{j}{1}-cB)') + ((obj.Positions{end-1, i}{j}{2}-cA)*(obj.Positions{end, i}{j}{2}-cB)') + ((obj.Positions{end-1, i}{j}{2}-cA)*(obj.Positions{end, i}{j}{2}-cB)');
                    [U S V] = svd(H);
                    R = V*U';
                    if det(R)<0
                        R(:,2) = R(:,2) * -1;
                    end

                    rt = t_2_xyzabc(R, 1);
                    angle = rt(3 - (j-1)); % gets 3 = x, 2 = y, 1 = z
                    if abs(angle) < 0.0001
                        angle = 0;
                    end

                    if abs(angle)  > 45
                        angle = 0;
                    end

                    RD{3} = [R(1,2) -R(1,1) 0 0; R(1,1) R(1,2) 0 0; 0 0 0 0; 0 0 0 0];
                    RD{2} = [R(3,1) 0 R(1,1) 0; 0 0 0 0; -R(1,1) 0 R(3,1) 0; 0 0 0 0];
                    RD{1} = [0 0 0 0; 0 R(2,3) -R(2,2) 0; 0 R(2,2) R(2,3) 0; 0 0 0 0];
                    
                    if j == 1
                        R = rotx(angle);
                    elseif j == 2
                        R = roty(angle);
                    elseif j == 3
                        R = rotz(angle);
                    end
                              
                    angle = deg2rad(angle);
                    omega{j} = (RD{j}*R')*angle*(1/timeStep);
                    
                    if j == 1
                        obj.OmegasX(idx, i) = omega{j}(3,2);
                    elseif j == 2
                        obj.OmegasY(idx, i) = omega{j}(1,3);
                    elseif j == 3
                        obj.OmegasZ(idx, i) = omega{j}(2,1);
                    end
                end
            end
        end
    end   
end

