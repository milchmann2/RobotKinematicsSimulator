function kr16_simulator()
% KR16_SIMULATOR for robotics course at FHS
%
clc;
close all;

% %Parameters for Trajectory Generation
t_ipo=0.1;
amax=20;
vc=50;
pose=3;
clear1=1;

robot = Robot();
%robot.bas(3,4)=100;
%robot.eff(1:3,4)=[0,0,100]';

% %Robot Trajectory in Euler Coordinates
% e{1}=[100,100,0,0,0,0,0];
% e{2}=[0,200, 0, 90, 0, 0, 1];
% e{3}=[0,300, 0, 90, 0, 0, 0];

%% top to bottom
% e{1}=[0,300, 0, 90, 0, 0, 0];
% e{2}=[300,0, 0, 90, 0, 0, 1];
% e{3}=[200,100,0,90,0,0,0];

% % straight down
% e{1}=[0,299, 0, 0, 0, 0, 0];
% e{2}=[0,300, 0, 90, 0, 0, 0];


% % straight down then to bottom arc
e{1}=[0,300, 0, 90, 0, 0, 0];
% e{2}=[0,250, 0, 90, 0, 0, 0];
% e{3}=[25,250, 0, 90, 0, 0, 0];
e{2}=[300,0, 0, 0, 90, 90, 0];

% 

% % % top to bottom without singularities
% e{1}=[10,150, 0, 75, 0, 0, 0];
% e{2}=[200,20, 0, 0, 0, 0, 0];
% e{3}=[0,300, 0, 90, 0, 0, 0];
% e{1}=[300,0, 0, 0, 0, 0, 0];
% e{2}=[0,300, 0, 90, 0, 0, 0];

% e{3}=[0,300, 0, 90, 0, 0, 0];


% % top to bottom without singularities ZZZZZ
% e{1}=[10,150, 50, 75, 0, 0, 0];
% e{2}=[200,20, 100, 0, 0, 0, 0];
% e{3}=[0,300, 0, 90, 0, 0, 0];


% e{2}=[500,-1000,1000,0,180,0,2*vc];
% e{1}=[930.91,0,1485,0,180,0,0];
% e{2}=[500,-1000,1000,0,180,0,2*vc];
% e{3}=[500,-1000,400,0,180,0,vc/2];
% e{4}=[500,-1000,1000,0,180,0,vc];
% e{5}=[500,-1100,1000,-90,90,0,vc];
% e{6}=[500,-1500,1000,-90,90,0,vc/2];
% e{7}=[500,-1100,1000,-90,90,0,vc];
% e{8}=[500,1000,1000,0,180,0,vc];
% e{9}=[500,1000,400,0,180,0,vc/2];
% e{10}=[500,1000,1000,0,180,0,vc];
% e{11}=[500,1100,1000,90,90,0,vc];
% e{12}=[500,1500,1000,90,90,0,vc/2];
% e{13}=[500,1100,1000,90,90,0,vc];
% e{14}=[930.91,0,1485,0,180,0,2*vc];

%**************************************************************************
%Implement your code to calculate the trajectory in axis coordinates here
%Use create_lin_seg-list, create_lin_int_vec and create_lin_path
%Calculate array q, so each column vector of q are the axis coordinates
%    for one interpolated point of the trajectory


%% This is a simple example of a trajectory in axis coordinates
% for i=1:90
%    q(:,i)=[i, -i, i]';
% end

% 
q_i = 1;
q = [0, 0, 0]';


gyro = Gyros();
        
for seg = 2:length(e)
    
    e1 = e{seg-1}(1:6);
    e2 = e{seg}(1:6);
    v = 2*vc;
    
    [tx ax] = robot.create_lin_seg_list(e1,e2,v,amax,t_ipo);
    [t, a, v, s] = robot.create_lin_intvec(tx, ax, t_ipo);
    ec = robot.create_lin_path(e1,e2,s);

    for i=1:length(ec)
        % SE(3) transformation matrix of the end-effector positions
        tg = xyzabc_2_t(ec{i}(1),ec{i}(2),ec{i}(3),ec{i}(4),ec{i}(5),ec{i}(6));
        tg(1:3, 4) = [ec{i}(1), ec{i}(2), ec{i}(4)]';
        % calculate the inverse kinematics for the pose
        q(:,q_i) = robot.kr16_rk(tg,robot.Base,robot.Endeffector,pose);         
        % points in space for the joint angles in respect to the base frame

        % calculate the forward kinematics to get the translation matrices
        % and thus the positions
        [T Tsub rot] = robot.fk_craig(q, robot);
        robot.DetermineLocalJointPositions(Tsub);
        % calculate joint rotation and translation matrices
        [rot1, tr] = gyro.CalculateRotations(q, robot.Joints.PositionsX, robot.Joints.PositionsY, robot.Joints.PositionsZ, robot);
 
        
        if (q_i == 1) % first position of trajectory segment
            p1z{q_i} = gyro.CalculatePosition(Tsub{1});
            p2z{q_i} = gyro.CalculatePosition(Tsub{2});
            p3z{q_i} = gyro.CalculatePosition(Tsub{3});
            p4z{q_i} = gyro.CalculatePosition(Tsub{4});
        else
            % angle change since the previous time step
            q_d = q(:,q_i) - q(:,q_i-1);
            % cc2 need to only send recent q in, not whole matrix
            [T Tsub, rot] = robot.fk_craig(q, robot);
            p1z{q_i} = gyro.CalculatePosition(Tsub{1});
            p2z{q_i} = gyro.CalculatePosition(Tsub{2});
            p3z{q_i} = gyro.CalculatePosition(Tsub{3});
            p4z{q_i} = gyro.CalculatePosition(Tsub{4});
        end

        if (q_i ~= 1)
            %**************************************************************
            % JOINT 1
            cA = (p1z{q_i-1}{1}+p1z{q_i-1}{2}+p1z{q_i-1}{3})/3;
            cB  = (p1z{q_i}{1}+p1z{q_i}{2}+p1z{q_i}{3})/3;            
            H = ((p1z{q_i-1}{1}-cA)*(p1z{q_i}{1}-cB)') + ((p1z{q_i-1}{2}-cA)*(p1z{q_i}{2}-cB)') + ((p1z{q_i-1}{2}-cA)*(p1z{q_i}{2}-cB)');
            [U S V] = svd(H);
            R = V*U';
            if det(R)<0
                R(:,2) = R(:,2) * -1;
            end
            
            rt = t_2_xyzabc(R, 1);
            a = rt(1);
            b = rt(2);
            c = rt(3);
            RDz = [R(1,2) -R(1,1) 0 0; R(1,1) R(1,2) 0 0; 0 0 0 0; 0 0 0 0];
            RDy = [R(3,1) 0 R(1,1) 0; 0 0 0 0; -R(1,1) 0 R(3,1) 0; 0 0 0 0];
            RDx = [0 0 0 0; 0 R(2,3) -R(2,2) 0; 0 R(2,2) R(2,3) 0; 0 0 0 0];
            Rx = rotx(c);
            Ry = roty(b);
            Rz = rotz(a);
            omegaX = (RDx*Rx')*c/(1/t_ipo);
            omegaY = (RDy*Ry')*b/(1/t_ipo);
            omegaZ = (RDz*Rz')*a/(1/t_ipo);
            gyro1x(q_i) = omegaX(3,2);
            gyro1y(q_i) = omegaY(1,3);
            gyro1z(q_i) = omegaZ(2,1)

            a =1 ;
            
            %**************************************************************
            %**************************************************************
            % JOINT 2
            cA = (p2z{q_i-1}{1}+p2z{q_i-1}{2}+p2z{q_i-1}{3})/3;
            cB  = (p2z{q_i}{1}+p2z{q_i}{2}+p2z{q_i}{3})/3;            
            H = ((p2z{q_i-1}{1}-cA)*(p2z{q_i}{1}-cB)') + ((p2z{q_i-1}{2}-cA)*(p2z{q_i}{2}-cB)') + ((p2z{q_i-1}{2}-cA)*(p2z{q_i}{2}-cB)');
            [U S V] = svd(H);
            R = V*U';
            
            t = -R*cA + cB;

            rt = t_2_xyzabc(R, 1);
            a = rt(1);
            b = rt(2);
            c = rt(3);
            RDz = [R(1,2) -R(1,1) 0 0; R(1,1) R(1,2) 0 0; 0 0 0 0; 0 0 0 0];
            RDy = [R(3,1) 0 R(1,1) 0; 0 0 0 0; -R(1,1) 0 R(3,1) 0; 0 0 0 0];
            RDx = [0 0 0 0; 0 R(2,3) -R(2,2) 0; 0 R(2,2) R(2,3) 0; 0 0 0 0];
            Rx = rotx(c);
            Ry = roty(b);
            Rz = rotz(a);
            omegaX = (RDx*Rx')*c/(1/t_ipo);
            omegaY = (RDy*Ry')*b/(1/t_ipo);
            omegaZ = (RDz*Rz')*a/(1/t_ipo);
            gyro2x(q_i) = omegaX(3,2);
            gyro2y(q_i) = omegaY(1,3);
            gyro2z(q_i) = omegaZ(2,1);


            
            %**************************************************************
            %**************************************************************
            % JOINT 3
            cA = (p3z{q_i-1}{1}+p3z{q_i-1}{2}+p3z{q_i-1}{3})/3;
            cB  = (p3z{q_i}{1}+p3z{q_i}{2}+p3z{q_i}{3})/3;            
            H = ((p3z{q_i-1}{1}-cA)*(p3z{q_i}{1}-cB)') + ((p3z{q_i-1}{2}-cA)*(p3z{q_i}{2}-cB)') + ((p3z{q_i-1}{2}-cA)*(p3z{q_i}{2}-cB)');
            [U S V] = svd(H);
            R = V*U';
            if det(R)<0
                R(:,2) = R(:,2) * -1;
            end
            
            rt = t_2_xyzabc(R, 1);
            a = rt(1);
            b = rt(2);
            c = rt(3);
            RDz = [R(1,2) -R(1,1) 0 0; R(1,1) R(1,2) 0 0; 0 0 0 0; 0 0 0 0];
            RDy = [R(3,1) 0 R(1,1) 0; 0 0 0 0; -R(1,1) 0 R(3,1) 0; 0 0 0 0];
            RDx = [0 0 0 0; 0 R(2,3) -R(2,2) 0; 0 R(2,2) R(2,3) 0; 0 0 0 0];
            Rx = rotx(c);
            Ry = roty(b);
            Rz = rotz(a);
            omegaX = (RDx*Rx')*c/(1/t_ipo);
            omegaY = (RDy*Ry')*b/(1/t_ipo);
            omegaZ = (RDz*Rz')*a/(1/t_ipo);
            gyro3x(q_i) = omegaX(3,2);
            gyro3y(q_i) = omegaY(1,3);
            gyro3z(q_i) = omegaZ(2,1);
            
            
            %**************************************************************
            %**************************************************************
            % JOINT 4
            cA = (p4z{q_i-1}{1}+p4z{q_i-1}{2}+p4z{q_i-1}{3})/3;
            cB  = (p4z{q_i}{1}+p4z{q_i}{2}+p4z{q_i}{3})/3;            
            H = ((p4z{q_i-1}{1}-cA)*(p4z{q_i}{1}-cB)') + ((p4z{q_i-1}{2}-cA)*(p4z{q_i}{2}-cB)') + ((p4z{q_i-1}{2}-cA)*(p4z{q_i}{2}-cB)');
            [U S V] = svd(H);
            R = V*U';
            if det(R)<0
                R(:,2) = R(:,2) * -1;
            end
            
            rt = t_2_xyzabc(R, 1);
            a = rt(1);
            b = rt(2);
            c = rt(3);
            RDz = [R(1,2) -R(1,1) 0 0; R(1,1) R(1,2) 0 0; 0 0 0 0; 0 0 0 0];
            RDy = [R(3,1) 0 R(1,1) 0; 0 0 0 0; -R(1,1) 0 R(3,1) 0; 0 0 0 0];
            RDx = [0 0 0 0; 0 R(2,3) -R(2,2) 0; 0 R(2,2) R(2,3) 0; 0 0 0 0];
            Rx = rotx(c);
            Ry = roty(b);
            Rz = rotz(a);
            omegaX = (RDx*Rx')*c/(1/t_ipo);
            omegaY = (RDy*Ry')*b/(1/t_ipo);
            omegaZ = (RDz*Rz')*a/(1/t_ipo);
            gyro4x(q_i) = omegaX(3,2);
            gyro4y(q_i) = omegaY(1,3);
            gyro4z(q_i) = omegaZ(2,1);
  
        else
            gyro1z(1) = 0;
            gyro2z(1) = 0;
            gyro3z(1) = 0;
            gyro4z(1) = 0;
        end
        q_i = q_i+1;
    end
end


            
%**************************************************************************
%assignin('base', 'gyroz', gyroz);
%assignin('base', 'realgyroz', realgyroz);

% initialize graph
figure(1);
clf;
 axis([-100 500 -100 500 -0.2 500]);
%axis([-2000 2000 -2000 2000 -0.2 2000]);
view([102,20]);
% view(2);
grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');

% draw robot path
%draw_robot_path(q,t_ipo,robot,100,clear1, p1, p2, p3, p1_2, p2_2, p3_2 ,p1_3, p2_3, p3_3, gyroz);


for i=1:size(p2z,2)
    p1{i} = p2z{i}{1};
    p2{i}= p2z{i}{2};
    p3{i}= p2z{i}{3};
    p1_2{i} = p3z{i}{1};
    p2_2{i} = p3z{i}{2};
    p3_2{i} = p3z{i}{3};
    p1_3{i} = p4z{i}{1};
    p2_3{i} = p4z{i}{2};
    p3_3{i} = p4z{i}{3};
end

gyros(:,1) = gyro1z';
gyros(:,2) = gyro2z';
gyros(:,3) = gyro3z';
gyros(:,4) = gyro4z';

accelplot = 0;
draw_robot_path(q,t_ipo,robot,100,clear1, p1, p2, p3, p1_2, p2_2, p3_2 ,p1_3, p2_3, p3_3, gyros, accelplot);


gyrosZ(:,1) = gyro1z';
gyrosZ(:,2) = gyro2z';
gyrosZ(:,3) = gyro3z';
gyrosZ(:,4) = gyro4z';

gyrosY(:,1) = gyro1y';
gyrosY(:,2) = gyro2y';
gyrosY(:,3) = gyro3y';
gyrosY(:,4) = gyro4y';

gyrosX(:,1) = gyro1x';
gyrosX(:,2) = gyro2x';
gyrosX(:,3) = gyro3x';
gyrosX(:,4) = gyro4x';

%plot gyro Z afterwards
subplot(3,1,1)
legend('1', '2', '3', '4', 'Position',[60, 60, 60, 60]);
plot(gyrosZ(:,1), 'r')
hold on;
plot(gyrosZ(:,2), 'b')
plot(gyrosZ(:,3), 'xg')
plot(gyrosZ(:,4), 'k')
valslength = size(gyrosZ(:,1), 1);
maxval = max(max(abs(gyrosZ)))*1.2;
axis([0 valslength -0.1 0.1])
title('GyrosZ');

%plot gyro Y afterwards
subplot(3,1,2)
legend('1', '2', '3', '4', 'Position',[60, 60, 60, 60]);
plot(gyrosY(:,1), 'r')
hold on;
plot(gyrosY(:,2), 'b')
plot(gyrosY(:,3), 'xg')
plot(gyrosY(:,4), 'k')
valslength = size(gyrosY(:,1), 1);
maxval = max(max(abs(gyrosY)))*1.2;
axis([0 valslength -maxval maxval])
title('GyrosY');

%plot gyro X afterwards
subplot(3,1,3)
legend('1', '2', '3', '4', 'Position',[60, 60, 60, 60]);
plot(gyrosX(:,1), 'r')
hold on;
plot(gyrosX(:,2), 'b')
plot(gyrosX(:,3), 'xg')
plot(gyrosX(:,4), 'k')
valslength = size(gyrosX(:,1), 1);
maxval = max(max(abs(gyrosX)))*1.2;
axis([0 valslength -0.1 0.1])
title('GyrosX');

