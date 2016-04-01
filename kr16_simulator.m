function kr16_simulator()
% KR16_SIMULATOR for robotics course at FHS
%
clc;
close all;
clear all;

% %Parameters for Trajectory Generation
% t_ipo=0.1;
% amax=1500;
% vc=500;
% pose=3;
% clear1=1;
 
robot = Robot();
%robot.bas(3,4)=100;
%robot.eff(1:3,4)=[0,0,100]';


%%%%%%%%%%%%%%%%%%%%%
%*********************
t_ipo=0.02;
amax=1500;
vc=500;
pose=3;
clear1=1;

robot=Robot();
robot.Base(3,4)=400;
% robot.Endeffector(1:3,4)=[0,0,155]';

% %Robot Trajectory in Euler Coordinates
% e{1}=[930.91,0,1485,0,180,0,0];
% % e{1}=[0,1000,1000,0,180,0,2*vc];
% % e{2}=[0,1000,1500,0,180,0,2*vc];
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

%cc4
% e{1}=[930.91,0,400,0,180,0,0];
% e{2}=[930.91,0,1600,0,180,0,2*vc];

%% wide movements
e{1}=[0, 1400,1800,0,0,-90,0];
e{2}=[1000,0,100,90,0,180,2*vc];
e{3}=[100,100,2200,0,0,0,2*vc];
% e{4}=[700, 700,700,0,0,-90,0];
% e{5}=[700,-700,100,-90,0,-90,2*vc];
% e{6}=[1400,100,1000,-90,90,0,2*vc];

% e{1}=[800, 1000,400,0,0,0,0];
%  e{2}=[1000,0,100,0,0,0,2*vc];
% e{3}=[800, 1400,350,0,0,-90,2*vc];
% e{4}=[1000,800,900,90,0,180,2*vc];


% e{5}=[1000,1000,500,90,0,0,2*vc];
% e{6}=[1000,500,1000,0,90,0,2*vc];
% e{7}=[1000,1000,500,0,0,0,2*vc];
% e{8}=[1000,500,1000,90,0,0,2*vc];
% 
% e{9}=[1000,1000,500,90,0,0,2*vc];
% e{10}=[500,1000,1000,90,0,90,2*vc];
% e{11}=[1000,1000,500,90,0,0,2*vc];
% e{12}=[500,1000,1000,0,90,0,2*vc];
% 
% e{13}=[1000,1000,500,10,10,10,2*vc];
% e{14}=[1000,1000,510,20,20,20,2*vc];
% e{15}=[1000,1000,520,30,30,30,2*vc];
% e{16}=[1000,1000,530,40,40,40,2*vc];
% 
% e{14}=[0, 1400,1800,0,0,-90,0];
% e{15}=[1000,0,100,90,0,180,2*vc];
% e{16}=[100,100,2200,0,0,0,2*vc];
% e{17}=[700, 700,700,0,0,-90,0];
% e{18}=[700,-700,100,-90,0,-90,2*vc];
% e{19}=[1400,100,1000,-90,90,0,2*vc];
% 
% 
% e{20}=[1000,1000,510,20,20,20,2*vc];
% e{21}=[700,-700,100,-90,0,-90,2*vc];
% e{22}=[1000,0,1000,90,0,180,2*vc];
% e{23}=[1400,100,1000,-90,90,0,2*vc];
% e{24}=[0, 1400,1000,0,0,-90,0];
% e{25}=[1000,500,1000,90,0,0,2*vc];
% e{26}=[500,1000,1000,0,90,0,2*vc];

% 
% e{1}=[0, 1400,1000,90,90,90,0];
% e{2}=[0, 1300,1100,180,90,90,0];
% e{3}=[0, 1400,1100,180,90,90,0];
% e{4}=[0, 1300,1100,180,180,180,0];
% e{5}=[0, 1400,1000,-45,180,-90,0];
% e{6}=[0, 1300,1100,-90,75,-100,0];
% e{7}=[0, 1400,1000,-110,130,-180,0];
% e{8}=[0, 1300,1100,180,-165,-75,0];
% e{9}=[0, 1400,1000,90,-180,-10,0];
% e{10}=[0, 1300,1000,0,0,0,0];
% e{11}=[0, 1400,1000,45,0,0,0];
% e{12}=[0, 1400,1000,90,0,0,0];
% e{13}=[0, 1400,1000,146,0,0,0];
% 
% e{1}=[0, 1400,1000,0,0,0,0];
% e{2}=[100, 1300,1100,0,0,0,0];
% e{3}=[0, 1400,1000,0,0,0,0];
% e{4}=[100, 1300,1100,0,0,0,0];
% e{5}=[0, 1400,1000,0,0,0,0];
% e{6}=[100, 1300,1100,0,0,0,0];

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
jointVelocities = zeros(1, joints);
time = 0;
position = zeros(1, joints);
qg = zeros(6,1);
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
%         [T Tsub rot] = robot.fk_craig(q, robot);
%         robot.DetermineLocalJointPositions(Tsub);
        % calculate joint rotation and translation matrices
%         [rot1, tr] = gyro.CalculateRotations(q, robot.Joints.PositionsX, robot.Joints.PositionsY, robot.Joints.PositionsZ, robot);
 
        
        if (q_i == 1) % first position of trajectory segment
            [T Tsub rot] = robot.fk_craig(q, robot);
            for i=1:joints
                if i == 1
                    p{q_i, i} = gyro.CalculatePosition([1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1]);
                else
                p{q_i, i} = gyro.CalculatePosition(Tsub{i-1});
                end
            end
            position(1,:) = deg2rad(q)';
            q1(1) = 0;
            q2(1) = 0;
            q3(1) = 0;
        else
            % angle change since the previous time step
            q_d = q(:,q_i) - q(:,q_i-1);
%             q_d = [0.1; 0.2; 0.3; 0.4; 0.5; 0.6];
            
            time(q_i) = time(q_i-1) + t_ipo;
            position(q_i,:) = deg2rad(q(:,q_i))';

            [T Tsub, rot] = robot.fk_craig(q, robot);
            for i=1:5
                Tsub{i}(1:3,:) =Tsub{i+1}(1:3,:);
            end
            for i=1:joints
                if i == 1
                    p{q_i, i} = gyro.CalculatePosition([1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1]);
                else
                p{q_i, i} = gyro.CalculatePosition(Tsub{i-1});
                end
            end
            if q_i > 1

                    aaaa = 1;
                    Tsub{6};
                
            end
            jointVelocities(q_i,:) = robot.CalculateJointVelocities(q_d, t_ipo);
            q1(q_i) = q_d(1)/t_ipo;
            q2(q_i) = (q_d(1)+q_d(2))/t_ipo;
            q3(q_i) = (q_d(1)+q_d(2)+q_d(3))/t_ipo;
        end

        if (q_i ~= 1)
            %**************************************************************
            % JOINT 1
            for i=1:joints
                for j=1:3   %x,y,z
                    cA = (p{q_i-1, i}{j}{1}+p{q_i-1, i}{j}{2}+p{q_i-1, i}{j}{3})/3;
                    cB  = (p{q_i, i}{j}{1}+p{q_i, i}{j}{2}+p{q_i, i}{j}{3})/3;            
                    H = ((p{q_i-1, i}{j}{1}-cA)*(p{q_i, i}{j}{1}-cB)') + ((p{q_i-1, i}{j}{2}-cA)*(p{q_i, i}{j}{2}-cB)') + ((p{q_i-1, i}{j}{2}-cA)*(p{q_i, i}{j}{2}-cB)');
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
                    % cc4
                    if q_i > 1
                        aaaa = 1;
                        if i == 6 && j == 1
                            aaaa = 1;
                        end
                    end
                    
                    if abs(angle)  > 45
                        angle = 0;
                    end
%                     
%                     a = rt(1);
%                     b = rt(2);
%                     c = rt(3);
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
                    omega{j} = (RD{j}*R')*angle*(1/t_ipo);
                    
                    if j == 1
                        gyros{q_i}{i}{j} = omega{j}(3,2);
                        gyrosX(q_i, i) = omega{j}(3,2);
                    elseif j == 2
                        gyros{q_i}{i}{j} = omega{j}(1,3);
                        gyrosY(q_i, i) = omega{j}(1,3);
                    elseif j == 3
                        gyros{q_i}{i}{j} = omega{j}(2,1);
                        gyrosZ(q_i, i) = omega{j}(2,1);
                    end
                end
            end
 
        else
            for i=1:joints
                for j=1:joints
                    gyros{q_i}{i}{j} = 0;
                end
            end
        end
        q_i = q_i+1;
    end
end


            
%**************************************************************************
%assignin('base', 'gyroz', gyroz);
%assignin('base', 'realgyroz', realgyroz);
q2 = gyrosZ(:,1);
assignin('base', 'q1', q1);
assignin('base', 'q2', q2);
assignin('base', 'gyrosZ', gyrosZ);
assignin('base', 'gyrosY', gyrosY);
assignin('base', 'gyrosX', gyrosX);

% initialize graph
figure(1);
clf;
%  axis([-100 500 -100 500 -0.2 500]);
axis([-2000 2000 -2000 2000 -0.2 2000]);
view([102,20]);
% view(90,0);

grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');

% cc2
% draw_robot_path(q,t_ipo,robot,100,clear1, p);


% figure(3)
% axis([0 1700 0 1700]);
% for j=i:size(p,1)
%     cla
%     p1 = p(j,:);
%     hold on
%     axis([-20 1700 -20 1700]);
%     for i=1:size(p1,2)
%         
%         plot3(p1{i}{1}(1, 4), p1{i}{1}(2, 4), p1{i}{1}(3, 4), 'bx');
%         plot3(p1{i}{2}(1, 4), p1{i}{2}(2, 4), p1{i}{2}(3, 4), 'go');
%         plot3(p1{i}{3}(1, 4), p1{i}{3}(2, 4), p1{i}{3}(3, 4), 'r*');
%     end
% end
    

for j=1:joints
    legendStrings{j} = int2str(j);
end
gyros{1} = gyrosX;
gyros{2} = gyrosY;
gyros{3} = gyrosZ;
gyrosTitle = {'Gyro X', 'Gyro Y', 'Gyro Z'};

figure
for i=1:3
    subplot(3,1,i)
    hold on;
    for j=1:joints
%         if j == 4
%             plot(gyros{i}(:,j));    
%         elseif j == 3
%             plot(gyros{i}(:,j), 'o');
%         else
            plot(gyros{i}(:,j));
%         end
    end
    legend(legendStrings);
    title(gyrosTitle{i});
end



% log.gyroX = [gyrosX; gyrosX; gyrosX; gyrosX; gyrosX; gyrosX];
% log.gyroY = [gyrosY; gyrosY; gyrosY; gyrosY; gyrosY; gyrosY];
% log.gyroZ = [gyrosZ; gyrosZ; gyrosZ; gyrosZ; gyrosZ; gyrosZ];
% jointVelocities = [jointVelocities;jointVelocities;jointVelocities;jointVelocities;jointVelocities;jointVelocities];
% log.velocity = jointVelocities./100;
% numModules = joints;
% log.time = [time;time;time;time;time;time];
% log.position = [position;position;position;position;position;position];

% cc5
log.gyroX = gyrosX ;
log.gyroY = gyrosY;
log.gyroZ = gyrosZ;
log.velocity = jointVelocities/10;
numModules = joints;
log.time = time;
log.position = position;

% Identify the base module based on the one that moved least
% The index of the smallest value in meanVels corresponds to the base
% module, becuase the IMU is before anything that moves.

% cc5
meanVels = mean( [ mean(abs(log.gyroX));
                   mean(abs(log.gyroY));
                   mean(abs(log.gyroZ)) ] );

wgx = mean(abs(log.gyroX))
wgy = mean(abs(log.gyroY))
wgz = mean(abs(log.gyroZ))
wvel = mean(abs(log.velocity))

for proximal = 1:numModules
    for distal = 1:numModules

        proximalModule = proximal; 
        distalModule = distal;
        proximalVels = [ log.gyroX(:,proximalModule) ...
                     log.gyroY(:,proximalModule) ...
                     log.gyroZ(:,proximalModule) + log.velocity(:,proximalModule)];

        distalVels = [ log.gyroX(:,distalModule) ...
                     log.gyroY(:,distalModule) ...
                     log.gyroZ(:,distalModule) ];

        % Rotate baseVels by the joint angle
        for i=1:length(log.time)
            proximalVels(i,:) = ...
                [R_z(-log.position(i,proximalModule)) * proximalVels(i,:)']';
        end

        % Dot-product all the velocities
        A = distalVels' * proximalVels;

        % Take the SVD
        [U,S,V] = svd(A);
        
        % Get the determinant of U*V
        S_SO3 = diag( [1, 1, sign(det(U*V))] );

        % Get the rotation
        R = V * S_SO3 * U';


        %% RESIDUAL ERROR
        errors(proximal,distal) = ...
            mean(mean((proximalVels - [R*distalVels']').^2));
        
        R_tests(:,:,proximal,distal) = R;
        
%         %% PLOTTING
%         figure(123);
%         plot(log.time,baseVels);
%         hold on;
%         plot(log.time,[R*nextVels']','--');
%         hold off;
%         
%         legend x y z x y z
%         title(['Velocities: ' num2str(proximal) '-->' num2str(distal) ]);
%         xlabel('time (sec)');
%         ylabel('angular velocity (rad/sec)');
%         drawnow;
%         pause;
        
    end
end

%% FINAL RESULT

errorThresh = .5;
connectionMatrix = errors < errorThresh

figure
imagesc(errors);
title('Velocity Matching Error');
xlabel('distal module');
ylabel('proximal module');
colorbar;

