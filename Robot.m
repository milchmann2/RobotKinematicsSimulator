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

            
            obj.DHP=[ 1,1,0,0,0,90;
                        1,1,0,200,0,0;
                        1,1,0,100,0,0
                        ];
                    
%             obj.DHP=[ 1,1,90,0,0,0;
%                         1,1,90,200,0,0;
%                         1,1,0,100,0,0
%                         ];        
                    
                    

            obj.Base=eye(4);
            obj.Endeffector=eye(4);
            obj.Endeffector(:,4) = [100; 0; 0; 1];
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
            
            dhp = robot.DHP;
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
            T = T*robot.Endeffector;
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

            [m,n] = size(bas);
            if (n ~= 4)|(m ~= 4)
                error('Base Frame must be 4 by 4');
            end

            [m,n] = size(eff);
            if (n ~= 4)|(m ~= 4)
                error('Tool Frame must be 4 by 4');
            end

            [m,n] = size(tg);
            if (n ~= 4)|(m ~= 4)
                error('Goal Frame must be 4 by 4');
            end

            if (pose<1)|(pose>8)
                error('Configuration must be between 1 and 8');
            end



            x = tg(1,4);
            y = tg(2,4);
            phi = tg(3,4);

            l1 = abs(obj.DHP(2,4));
            l2 = abs(obj.DHP(3,4));
            l3 = 100;

            c2 = (x^2 + y^2 - l1^2 - l2^2) / (2*l1*l2);
            s2_1 = sqrt(1-c2^2);
            s2_2 = -s2_1;

            theta2_1 = rad2deg(atan2(s2_1, c2));
            theta2_2 = rad2deg(atan2(s2_2, c2));

            c1_1 = (x*(l1+l2*c2) + y*l2*s2_1)/(l1^2+l2^2+2*l1*l2*c2);
            c1_2 = (x*(l1+l2*c2) + y*l2*s2_1)/(l1^2+l2^2+2*l1*l2*c2);
            s1_1 = (y*(l1+l2*c2)-x*l2*s2_1)/(l1^2+l2^2+2*l1*l2*c2);
            s1_2 = (y*(l1+l2*c2)-x*l2*s2_2)/(l1^2+l2^2+2*l1*l2*c2);
            theta1 = rad2deg((atan2(s1_1,c1_1)));

            theta3 = phi - theta1 - theta2_1;

            q = [theta1 theta2_1 theta3]';
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

            dist_vect = e2(1:3) - e1(1:3);  %Abstandsvektor
            s_tot = norm(dist_vect);    %Weg zwischen Punkt1 und Punkt2
            tb = t_ipo * ceil((vc / amax) / t_ipo); %Beschleunigungszeit auf auf Vc (angepaßt an t_ipo)
            sb = amax/2 * tb^2; %Weg bis zur Vc
            if 2 * sb >= s_tot
                sb = s_tot / 2; %Beschleunigungsweg = halber Gesamtweg
                tb = sqrt(sb / (amax/2));   %errechnete Beschleunigungszeit
                tb = t_ipo * ceil(tb / t_ipo);   %Beschleunigungszeit angepaßt auf t_ipo
                tx = [tb 0 tb]; 
                a = (sb / tb^2) * 2;    %tatsächliche Beschleunigung
                ax = [a 0 -a];

            else
                %Beschleunigung und Verzögerungphase
                tx = [tb 0 tb];
                a = vc / tx(1);
                ax = [a 0 -a];

                %Lineare Phase
                s_lin = s_tot - 2 * sb; %Weg der linearen Phase
                t_lin  = s_lin / vc; %Zeit für diese Strecke
                tx(2) = t_ipo * ceil(t_lin / t_ipo);
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

            t_tot = sum(t_seg);
            segment_count = floor(t_tot/t_ipo+1);

            t = zeros(segment_count,1);

            for i = 1:segment_count
                t(i) = (i-1)*t_ipo;
            end

            a = zeros(segment_count,1);

            i=1;
            for j = 0:t_ipo:(t_seg(1)-t_ipo)
                a(i)=a_seg(1);
                i=i+1;
            end
            for j = t_seg(1):t_ipo:(t_seg(1)+t_seg(2)-t_ipo)
                a(i)=a_seg(2);
                i=i+1;
            end
            for j = (t_seg(1)+t_seg(2)):t_ipo:(t_seg(1)+t_seg(2)+t_seg(3)-t_ipo)
                a(i)=a_seg(3);
                i=i+1;
            end

            a(i)=0;

            v = zeros(segment_count,1);
            s = zeros(segment_count,1);

            for i = 2:segment_count
                v(i)=v(i-1)+a(i-1)*t_ipo;
                s(i)=s(i-1)+v(i)*t_ipo;
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

            s_tot = s(end);

            %ec = cell(6);

            for i = 1:length(s)

                x = e1(1)+(e2(1)-e1(1))*s(i)/s_tot;
                y = e1(2)+(e2(2)-e1(2))*s(i)/s_tot;
                z = e1(3)+(e2(3)-e1(3))*s(i)/s_tot;
                a = e1(4)+(e2(4)-e1(4))*s(i)/s_tot;
                b = e1(5)+(e2(5)-e1(5))*s(i)/s_tot;
                c = e1(6)+(e2(6)-e1(6))*s(i)/s_tot;

                ec{i} = [x,y,z,a,b,c];

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
        
    end
    
end

