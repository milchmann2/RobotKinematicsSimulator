function kr16_simulator()
% KR16_SIMULATOR for robotics course at FHS
%
clc;
close all;


% %Parameters for Trajectory Generation
% t_ipo=0.1;
% amax=1500;
% vc=500;
% pose=3;
% clear1=1;
 
%robot.bas(3,4)=100;
%robot.eff(1:3,4)=[0,0,100]';


%%%%%%%%%%%%%%%%%%%%%
%*********************
t_ipo=0.02;
amax=1500;
vc=500;
pose=3;
clear1=1;



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
e{4}=[700, 700,700,0,0,-90,0];
e{5}=[700,-700,100,-90,0,-90,2*vc];
e{6}=[1400,100,1000,-90,90,0,2*vc];

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

% e{14}=[0, 1400,1800,0,0,-90,0];
% e{15}=[1000,0,100,90,0,180,2*vc];
% e{16}=[100,100,2200,0,0,0,2*vc];
% e{17}=[700, 700,700,0,0,-90,0];
% e{18}=[700,-700,100,-90,0,-90,2*vc];
% e{19}=[1400,100,1000,-90,90,0,2*vc];


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
q = [0, 0, 0, 0, 0, 0]';

robot = Robot(q);
robot.Base(3,4)=400;
joints = robot.NumberOfJoints;
jointVelocities = zeros(1, joints);


directJointControl = 0;


q_i = 2;
for seg = 2:length(e)
    
    e1 = e{seg-1}(1:6);
    e2 = e{seg}(1:6);
    v = 2*vc;
    
    [tx ax] = robot.create_lin_seg_list(e1,e2,v,amax,t_ipo);
    [t, a, v, s] = robot.create_lin_intvec(tx, ax, t_ipo);
    ec = robot.create_lin_path(e1,e2,s);

    for i=1:length(ec)

        tg = xyzabc_2_t(ec{i}(1), ec{i}(2), ec{i}(3), ec{i}(4), ec{i}(5), ec{i}(6));
        q(:,q_i) = robot.kr16_rk(tg,robot.Base,robot.Endeffector,pose);         
        

        if directJointControl == 1
            if q_i == 1
                q(:, 1) = q_f(1,:)';
            else 
                q(:, q_i) = q_f(q_i,:)'; 
            end
        end
        
        q_d = q(:,q_i) - q(:,q_i-1);

        robot.Time(q_i) = robot.Time(q_i-1) + t_ipo;    % t_ipo(q_i)
        robot.JointPositions(q(:,q_i));
        [T Tsub rot] = robot.fk_craig(q, robot);
        robot.CalculateGyroPosition(Tsub);
        jointVelocities(q_i,:) = robot.CalculateJointVelocities(q_d, t_ipo);   % t_ipo(q_i)
        robot.CalculateAngularVelocities(t_ipo); % t_ipo(q_i)
        q_i = q_i+1;        
       
    end
end


            
%**************************************************************************
%assignin('base', 'gyroz', gyroz);
%assignin('base', 'realgyroz', realgyroz);
% assignin('base', 'gyrosZ', gyrosZ);

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
% draw_robot_path(q,t_ipo,robot,100, 1, robot.Gyros.Positions);


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
gyros{1} = robot.Gyros.OmegasX;
gyros{2} = robot.Gyros.OmegasY;
gyros{3} = robot.Gyros.OmegasZ;
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
log.gyroX = robot.Gyros.OmegasX;
log.gyroY = robot.Gyros.OmegasY;
log.gyroZ = robot.Gyros.OmegasZ;
log.velocity = jointVelocities;
log.time = robot.Time;
log.position = robot.Joints.PositionsRad;
log.numModules = joints;

%jointDetectionDave(log);
jointDetectionSingle(log);
jointDetectionDave(log);
