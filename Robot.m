classdef Robot < handle
    %ROBOT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        DHP
        Base
        Endeffector
        NumberOfJoints = 0;
        Joints;   
    end
    
    methods


        function obj = Robot()
            % robot = kr16_robot()
            % creates a structure with robot parameters for the KUKA KR16-2 robot
            % dimensions are in mm, degrees in deg, velocitis in m/s or deg/s
            % accelerations in m/s^2 or deg/s^2
            %
            %  robot.dhp  DH parameters, [n 6] Array
            %             colums:    type   sign  alpha  a    d    theta 
            %                        (1/2) (1/-1)
            %                         1...rotational axis
            %                         2...translational axis
            %
            %  robot.eff:  tool frame (in flange coordinates)
            %              [4 4] Array
            %
            %  robot.bas:  base frame (in world coordinates)
            %              [4 4] Array
            % 
            %   ALL ANGLES IN DEG!!
            %
            %************************************************
            %  DH-Parameter for KR16 Robot                  *
            %************************************************

%             % RRR Planar Movements
%             obj.DHP=[ 1,1,90,0,0,0;
%                         1,1,0,200,0,0;
%                         1,1,0,100,0,0
%                         ];
                    
%             obj.DHP=[ 1,1,90,0,0,0;
%                         1,1,90,200,0,0;
%                         1,1,0,100,0,0
%                         ];        

            obj.DHP= [    
                                    1, 1,   180,    0,      -675,   0;
                                    1, 1,   90,     260,    0,      0;
                                    1, 1,   0,      680,    0,      90;
                                    1, 1,   -90,    35,     -670,   0;
                                    1, 1,   90,     0,      0,          0;
                                    1, -1,  90,     0,      115,    0];

            obj.Base = eye(4);
            obj.Endeffector = eye(4);


                    


            obj.NumberOfJoints = size(obj.DHP, 1);
            %obj.Gyros = 
        end
        
        function [T, T_sub, rot] = fk_craig(obj, q,robot)
            % Calculates the foreward kinematics (tool frame) of a 
            % robot in world coordinates
            % considers base and tool frame
            % uses CRAIG frame assignments
            %
            %
            % Input Parameter:
            % q:        column vector with joint values,[n 1] Array
            % robot:    structure with robot parameters
            %           robot.dh   DH parameter, [n 6] Array
            %           colums:    type   sign  alpha  a    d    theta 
            %                      (1/2) (1/-1)
            %                       1...rotational axis
            %                       2...translational axis
            %
            %           robot.eff:  tool frame (in flange coordinates)
            %           [4 4] Array
            %
            %           robot.bas base frame (in world coordinates)
            %           [4 4] Array
            % 
            %   ALL ANGLES IN DEG!!
            %
            % Return Parameters:
            % T:    tool frame; 
            %
            %error(nargchk(2,2,nargin));
            
            T=zeros(4);

            dhp = obj.DHP;
            % calculate the full foward kinematics starting from the baseframe up to
            % the end effector
            q = [q(:,end); 0];
            dhp = [robot.DHP; 0 0 0 obj.Endeffector(1, 4) 0 0];
            T = robot.Base;
            T_sub = cell(size(dhp,1),1);
            for i = 1:size(dhp, 1)
                alpha = dhp(i,3);
                a = dhp(i,4);
                d = dhp(i,5);
                theta = dhp(i,6)+q(i)*dhp(i,2);
                T_temp = dh_trafo_craig(alpha,a,d,theta);
                T = T*T_temp;
                T_sub{i} = T;
                rot{i} = zeros(4,4);
                rot{i}(4,4) = 1;
                rot{i}(1:3,1:3) = T(1:3, 1:3);
            end

            T = T * obj.Endeffector;
        end
        
        function q = kr16_rk(obj, tg,bas,eff,pose)
            %Backwards Kinematics 
            %
            %	Q = KR16_RK(TG,BAS,EFF,POSE)  calculates the joint angles Q of the Backward Kinematic Solution
            %	of the Kuka KR-16/2 manipulator for a given goal transformation T and
            %	desired configuration POSE
            %   Limits of axis angles are currently not taken into account
            %	
            %	tg: goal frame
            %   bas: base frame
            %   eff: tool frame
            %   pose: is the desired robot configuration
            % 
            %   pose    q1      Elbow   wrist
            %    1      front   up      pos
            %    2      front   up      neg
            %    3      front   down    pos
            %    4      front   down    neg
            %    5      back    down    pos
            %    6      back    down    neg
            %    7      back    up      pos
            %    8      back    up      neg
            %
            %error(nargchk(4,4,nargin));

%             [m,n] = size(bas);
%             if (n ~= 4)|(m ~= 4)
%                 error('Base Frame must be 4 by 4');
%             end
% 
%             [m,n] = size(eff);
%             if (n ~= 4)|(m ~= 4)
%                 error('Tool Frame must be 4 by 4');
%             end
% 
%             [m,n] = size(tg);
%             if (n ~= 4)|(m ~= 4)
%                 error('Goal Frame must be 4 by 4');
%             end
% 
%             if (pose<1)|(pose>8)
%                 error('Configuration must be between 1 and 8');
%             end



%             x = tg(1,4);
%             y = tg(2,4);
%             phi = tg(3,4);
% 
%             l1 = abs(obj.DHP(2,4));
%             l2 = abs(obj.DHP(3,4));
%             l3 = 100;
% 
%             c2 = (x^2 + y^2 - l1^2 - l2^2) / (2*l1*l2);
%             s2_1 = sqrt(1-c2^2);
%             s2_2 = -s2_1;
% 
%             theta2_1 = rad2deg(atan2(s2_1, c2));
%             theta2_2 = rad2deg(atan2(s2_2, c2));
% 
%             c1_1 = (x*(l1+l2*c2) + y*l2*s2_1)/(l1^2+l2^2+2*l1*l2*c2);
%             c1_2 = (x*(l1+l2*c2) + y*l2*s2_1)/(l1^2+l2^2+2*l1*l2*c2);
%             s1_1 = (y*(l1+l2*c2)-x*l2*s2_1)/(l1^2+l2^2+2*l1*l2*c2);
%             s1_2 = (y*(l1+l2*c2)-x*l2*s2_2)/(l1^2+l2^2+2*l1*l2*c2);
%             theta1 = rad2deg((atan2(s1_1,c1_1)));
% 
%             theta3 = phi - theta1 - theta2_1;
% 
%             q = [theta1 theta2_1 theta3]';

            
            %generate roboter

            dhp = obj.DHP;

            % calculate the position of the center spherical wrist
            df = abs(dhp(6, 5));
            P04org = inv(bas) * tg * inv(eff) * [0; 0; -df; 1];

            p4x = P04org(1);
            p4y = P04org(2);
            p4z = P04org(3);

            % theta1-3 according to formulas on the slides
            % theta1
            if(pose == 1 || pose == 2 || pose == 3 || pose == 4)
                theta1 = rad2deg(atan2(-p4y, p4x));
            else
                theta1 = rad2deg(atan2(p4y, -p4x));
            end

            % theta2
            r = p4x * cosd(theta1) - p4y * sind(theta1);

            a1 = abs(dhp(2, 4));
            d1 = abs(dhp(1, 5));
            a2 = abs(dhp(3, 4));
            a3 = abs(dhp(4, 4));

            k1 = r - a1;
            k2 = d1 - p4z;
            d4 = abs(dhp(4, 5));
            k3 = ((r - a1)^2 + (d1 - p4z)^2 + a2^2 - a3^2 - d4^2) / (2 * a2);

            if(pose == 1 || pose == 2 || pose == 7 || pose == 8)
                theta2 = rad2deg(atan2(k2,k1) + atan2(sqrt(k1^2 + k2^2 - k3^2),k3));
            else
                theta2 = rad2deg(atan2(k2,k1) - atan2(sqrt(k1^2 + k2^2 - k3^2),k3));
            end

            % theta3
            k4 = -a1 - a2 * cosd(theta2) + r;
            k5 = -a2 * sind(theta2) + d1 - p4z;

            theta3 = -theta2 + rad2deg(atan2(d4 * k5 - a3 * k4, d4 * k4 + a3 * k5));

            % change of orientation caused by the spherical wrist R3F
            % forward kinematics of the first three axes
            dhp = obj.DHP;
            T03 = obj.Base;
            q = [theta1; theta2; theta3];
            for i = 1:3
                alpha = dhp(i, 3);
                a = dhp(i, 4);
                d = dhp(i, 5);
                theta = dhp(i, 6) + q(i) * dhp(i, 2);    % angle * sign
                T_current = dh_trafo_craig(alpha, a, d, theta);
                T03 = T03 * T_current;    
            end

            T6F = [-1 0 0 0; 0 1 0 0; 0 0 -1 -df; 0 0 0 1]; % from the script
            T36 = inv(T03) * inv(bas) * tg * inv(eff) * inv(T6F);

            if(pose == 1 || pose == 3 || pose == 5 || pose == 7)
                theta5 = rad2deg(atan2(sqrt(T36(2, 1)^2 + T36(2, 2)^2), T36(2, 3)));
                theta4 = rad2deg(atan2(T36(3, 3) / sind(theta5), -T36(1,3) / sind(theta5)));
                theta6 = rad2deg(atan2(-T36(2, 2) / sind(theta5), T36(2, 1) / sind(theta5)));
            else
                theta5 = rad2deg(atan2(-sqrt(T36(2, 1)^2 + T36(2, 2)^2), T36(2, 3)));
                theta4 = rad2deg(atan2(T36(3, 3) / sind(theta5),- T36(1, 3) / sind(theta5)));
                theta6 = rad2deg(atan2(-T36(2, 2) / sind(theta5), T36(2, 1) / sind(theta5)));
            end

            q = [theta1 theta2 theta3 theta4 theta5 theta6]';   

        end
        
        function [tx, ax] = create_lin_seg_list(obj, e1,e2,vc,amax,t_ipo)
            % [tx, ax] = create_lin_seg_list(e1,e2,amx,t_ipo)
            % generates the segment list for a single leg of a linear trajectory
            %
            % ta and tc are syncronized with t_ipo if t_ipo>0
            %
            % e1: start position (euler coordinates)
            % e2: end position (euler coordinates)
            % vc: speed in the linear segment of the trajectory
            % amax: maximum acceleration to be used
            % t_ipo: interpolation clock
            % 
            % tx: column vector with the duration of each segment
            % ax: column vector with the acceleration used for each segment


            %error(nargchk(5,5,nargin));

            %****************************************************************
            %Implement your code here


            tx=[0 0 0]';
            ax=[0 0 0]';

            % calculate the eucledian norm
            s_totalal = norm(e2 - e1);

            % check if we reach the halfpoint of our way before we reach the our
            % maximum velocity
            % determines if we have trapezoidal or triangular profile
            s_critical = vc^2 / amax;
            if s_critical >= s_totalal    %triangular
                t_a_total =  sqrt(s_totalal / amax);     

                time_difference = mod(t_a_total, t_ipo);
                if time_difference ~= 0
                    delta = t_ipo - time_difference;
                    t_a_total = t_a_total + delta;
                    t_d_total = t_a_total;
                    t_c_total = 0;
                    amax = s_totalal / (t_a_total^2 + t_a_total * t_c_total);
                end
                tx = [ t_a_total 0 t_a_total]        % t_a_total = t_d_total
                ax = [amax 0 -amax];    

            else                                % trapezoidal
                t_a_total = vc / amax;    
                s_a = amax * t_a_total^2 / 2;
                s_d = s_a;
                s_c = s_totalal - s_a - s_d;
                t_c_total = s_c / vc;

                time_difference = mod(t_a_total, t_ipo);
                if time_difference ~= 0        
                     delta = t_ipo - time_difference;
                    t_a_total = t_a_total + delta;
                    t_d_total = t_a_total;
                end
                %t_c_total = 0;
                time_difference = mod(t_c_total, t_ipo);
                if time_difference ~= 0
                    delta = t_ipo - time_difference;
                    t_c_total = t_c_total + delta;
                end 

                 tx = [t_a_total t_c_total t_a_total];   % t_a_total = t_d_total
                 ax = [amax 0 -amax];
            end
        end
        
        function [t, a, v, s] = create_lin_intvec(obj, t_seg, a_seg, t_ipo)
            % [t, a, v, s] = create_lin_intvec(t_seg, a_seg, t_ipo)
            % generates a linear interploation vector s(t) from the segments of a 
            % trajectory
            %
            % t_seg: columnt vector with the duration of each segment
            % a_seg: column vector with the acceleration of each segment
            % t_ipo: interpolation clock
            % 
            % the values of t must be multiples of t_ipo!!!!!!!

            %error(nargchk(3,3,nargin));


            %****************************************************************
            %Implement your code here


                t=[0 0 0 0 0];
            a=[0 0 0 0 0];
            v=[0 0 0 0 0];
            s=[0 0 0 0 0];

            t_tot = sum(t_seg);
            segment_count = floor(t_tot/t_ipo+1);

            t = zeros(segment_count,1);
            for i = 1:segment_count
                t(i) = (i-1)*t_ipo;
            end

            a = zeros(segment_count, 1);
            i = 1;
            for j = 0:t_ipo:(t_seg(1)-t_ipo)
                a(i) = a_seg(1);
                i= i+1;
            end
            for j = t_seg(1):t_ipo:(t_seg(1)+t_seg(2)-t_ipo)
                a(i) = a_seg(2);
                i= i+1;
            end
            for j = (t_seg(1)+t_seg(2)):t_ipo:(t_seg(1)+t_seg(2)+t_seg(3)-t_ipo)
                a(i) = a_seg(3);
                i = i+1;
            end

            v = zeros(segment_count,1);
            s = zeros(segment_count,1);
            for i = 2:segment_count
                v(i)=v(i-1) + a(i-1) * t_ipo;
                s(i)=s(i-1) + v(i) * t_ipo;
            end     
        end 
        
        function ec = create_lin_path(obj, e1,e2,s)
            % ec = create_lin_path(e1,e2,s)
            % generates a cell array containing the vectors of euler coordinates for a
            % path
            % %
            % e1: start position (euler coordinates)
            % e2: end position (euler coordinates)
            % s: interpolation vector
            % 
            % ec: cell array containing vectors of euler coordinates for the
            % interpolated path


            %error(nargchk(3,3,nargin));

            %****************************************************************
            %Implement your code here

            ec{1}=[0 0 0 0 0 0];
            ec{2}=[0 0 0 0 0 0];


            s_total = s(end);

            for i = 1:length(s) 
                x = e1(1) + (e2(1) - e1(1)) * s(i) / s_total;
                y = e1(2) + (e2(2) - e1(2)) * s(i) / s_total;
                z = e1(3) + (e2(3) - e1(3)) * s(i) / s_total;
                a = e1(4) + (e2(4) - e1(4)) * s(i) / s_total;
                b = e1(5) + (e2(5) - e1(5)) * s(i) / s_total;
                c = e1(6) + (e2(6) - e1(6)) * s(i) / s_total;

                ec{i} = [x, y, z, a, b, c];
            end
        end
        
        function DetermineLocalJointPositions(obj, T)
            % TODO needs to be changed, takes fixed EE, and only x+y plane
%             jointsAndEndEffector = [obj.DHP; 0 0 0 obj.Endeffector(1, 4) 0 0];
%             q = [q(:,end); 0];
%             obj.Joints.PositionsX(1) = obj.Base(1,4);
%             obj.Joints.PositionsY(1) = obj.Base(2,4);
%             
%             for i=2:obj.NumberOfJoints+1
%                 jointLength = jointsAndEndEffector(i, 4);
%                 combinedAngle = sum(q(1:i-1));
%                 px = jointLength*cosd(combinedAngle);
%                 py = jointLength*sind(combinedAngle);
%                 obj.Joints.PositionsX(i) = px;
%                 obj.Joints.PositionsY(i) = py;
%                 obj.Joints.PositionsX = obj.Joints.PositionsX';
%                 obj.Joints.PositionsY = obj.Joints.PositionsY';
%             end
            obj.Joints.PositionsX = zeros(size(T, 1), 1);
            for i=1:size(T, 1)
                abcxyz = t_2_xyzabc(T{i}, 1);
                obj.Joints.PositionsX(i, 1) = abcxyz(4);
                obj.Joints.PositionsY(i, 1) = abcxyz(5);
                obj.Joints.PositionsZ(i, 1) = abcxyz(6);
                % cc2
                obj.Joints.AnglesX(i, 1) = abcxyz(3);
                obj.Joints.AnglesY(i, 1) = abcxyz(2);
                obj.Joints.AnglesZ(i, 1) = abcxyz(1);
            end
            for i=size(obj.Joints.AnglesX, 1):-1:2
                obj.Joints.AnglesX(i, 1) = obj.Joints.AnglesX(i, 1) - obj.Joints.AnglesX(i-1, 1);
                obj.Joints.AnglesY(i, 1) = obj.Joints.AnglesY(i, 1) - obj.Joints.AnglesY(i-1, 1);
                obj.Joints.AnglesZ(i, 1) = obj.Joints.AnglesZ(i, 1) - obj.Joints.AnglesZ(i-1, 1);
            end
        end
        
        function jointVelocities = CalculateJointVelocities(obj, dq, dt)
            jointVelocities = dq./dt; 
        end
        
    end
    
end

