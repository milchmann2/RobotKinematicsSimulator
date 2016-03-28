function kr16_simulator()
% KR16_SIMULATOR for robotics course at FHS
%
clc;
close all;

% %Parameters for Trajectory Generation
t_ipo=0.01;
amax=20;
vc=50;
pose=3;
clear1=1;
 
robot = Robot();
%robot.bas(3,4)=100;
%robot.eff(1:3,4)=[0,0,100]';


%%%%%%%%%%%%%%%%%%%%%
%*********************
t_ipo=0.01;
amax=1500;
vc=500;
pose=3;
clear1=1;

robot=Robot();
robot.Base(3,4)=400;
% robot.Endeffector(1:3,4)=[0,0,155]';

% %Robot Trajectory in Euler Coordinates
% e{1}=[930.91,0,1485,0,180,0,0];
e{1}=[0,1000,1000,0,180,0,2*vc];
e{2}=[0,1000,1500,0,180,0,2*vc];
% e{3}=[0,-1500,1000,0,180,0,vc/2];
% e{4}=[500,-1000,1000,0,180,0,vc];
% e{5}=[500,-1100,1000,-90,90,0,vc];
% e{6}=[500,-1500,1000,-90,90,0,vc/2];


% e{1}=[50,0, 200, 0, 0, 0, 0];
% e{2}=[100,50, 200, 0, 0, 0, 0];
% e{3}=[0,150, 200, 0, 0, 0, 0];

% e{1}=[0,300, 0, 90, 0, 0, 0];
% % e{2}=[0,250, 0, 90, 0, 0, 0];

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
% e{1}=[0,300, 0, 90, 0, 0, 0];
% % e{2}=[0,250, 0, 90, 0, 0, 0];
% % e{3}=[25,250, 0, 90, 0, 0, 0];
% e{2}=[300,0, 0, 0, 90, 90, 0];

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

% e{1}=[930.91,0,1485,0,180,0,0];
% e{2}=[500,-1000,1000,0,180,0,2*vc];
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
q = [0, 0, 0, 0, 0, 0]';


gyro = Gyros();

joints = 6;
gyrosZ = zeros(1, joints);
gyrosY = zeros(1, joints);
gyrosX = zeros(1, joints);
p{1, joints} = 0;

for seg = 2:length(e)
    
    e1 = e{seg-1}(1:6);
    e2 = e{seg}(1:6);
    v = 2*vc;
    
    [tx ax] = robot.create_lin_seg_list(e1,e2,v,amax,t_ipo);
    [t, a, v, s] = robot.create_lin_intvec(tx, ax, t_ipo);
    ec = robot.create_lin_path(e1,e2,s);

    for i=1:length(ec)
        % SE(3) transformation matrix of the end-effector positions
        tg = xyzabc_2_t(ec{i}(1), ec{i}(2), ec{i}(3), ec{i}(4), ec{i}(5), ec{i}(6));

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
            for i=1:joints
                p{q_i, i} = gyro.CalculatePosition(Tsub{i});
            end
        else
            % angle change since the previous time step
            q_d = q(:,q_i) - q(:,q_i-1);
            % cc2 need to only send recent q in, not whole matrix
            [T Tsub, rot] = robot.fk_craig(q, robot);
            for i=1:joints
                p{q_i, i} = gyro.CalculatePosition(Tsub{i});
            end
        end

        if (q_i ~= 1)
            %**************************************************************
            % JOINT 1
            for i=1:joints
                cA = (p{q_i-1, i}{1}+p{q_i-1, i}{2}+p{q_i-1, i}{3})/3;
                cB  = (p{q_i, i}{1}+p{q_i, i}{2}+p{q_i, i}{3})/3;            
                H = ((p{q_i-1, i}{1}-cA)*(p{q_i, i}{1}-cB)') + ((p{q_i-1, i}{2}-cA)*(p{q_i, i}{2}-cB)') + ((p{q_i-1, i}{2}-cA)*(p{q_i, i}{2}-cB)');
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
                gyrosX(q_i, i) = omegaX(3,2);
                gyrosY(q_i, i) = omegaY(1,3);
                gyrosZ(q_i, i) = omegaZ(2,1);
            end
 
        else
            for i=1:joints
                gyrosZ(i, 1) = 0;
            end
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
%  axis([-100 500 -100 500 -0.2 500]);
axis([-2000 2000 -2000 2000 -0.2 2000]);
view([102,20]);
% view(2);
grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');


draw_robot_path(q,t_ipo,robot,100,clear1, p);

for j=1:joints
    legendStrings{j} = int2str(j);
end
gyros{1} = gyrosX;
gyros{2} = gyrosY;
gyros{3} = gyrosZ;
gyrosTitle = {'Gyro X', 'Gyro Y', 'Gyro Z'};

for i=1:3
    subplot(3,1,i)
    hold on;
    for j=1:joints
        if j == 4
            plot(gyros{i}(:,j), 'x');    
        elseif j == 3
            plot(gyros{i}(:,j), 'o');
        else
            plot(gyros{i}(:,j));
        end
    end
    legend(legendStrings);
    title(gyrosTitle{i});
end

