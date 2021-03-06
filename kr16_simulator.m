function kr16_simulator()
% KR16_SIMULATOR for robotics course at FHS
%
clc;
close all;
% clear all;
if ~exist('HebiLookup', 'class')
    display('adding HEBI library');
    addpath('hebi');
    addpath(genpath('C:\Users\Milchmann\Dropbox\01 - FH\Master''s Thesis\07 - Simulator\Simulator\matlab_SEA'));
end

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



%% wide movements




%% This is a simple example of a trajectory in axis coordinates
% for i=1:90
%    q(:,i)=[i, -i, i]';
% end

% 


robot = Robot();
robot.Base(3,4)=400;
joints = robot.NumberOfJoints;
jointVelocities = zeros(1, joints);
q = zeros(joints, 1);



angleControl = 1;
useLogFile = 1;
    
if angleControl == 0
    e{1}=[0, 1400,1800,0,0,-90,0];
    e{2}=[1000,0,100,90,0,180,2*vc];
    e{3}=[100,100,2200,0,0,0,2*vc];
    e{4}=[700, 700,700,0,0,-90,0];
    e{5}=[700,-700,100,-90,0,-90,2*vc];
    e{6}=[1400,100,1000,-90,90,0,2*vc];
end

if angleControl == 1
%     e = zeros(1,2);
%     q_f = (struct2array(load('q.mat')));
%     q_f = rad2deg(q_f);
%     q_f = [q_f];
%     t_ipo = struct2array(load('t_d.mat'));
%     t_ipo = [t_ipo];



    if useLogFile == 1
        log = struct(HebiUtils.convertGroupLog('log2','view','full'));
        q_f = log.position;
        joints = size(log.position, 2);
        jointVelocities = zeros(1, joints);
        t_ipo = zeros(1, size(log.position, 1));
        q = zeros(joints, 1);
        for i=2:size(log.time,1)
            t_ipo(i) = log.time(i) - log.time(i-1);
        end
        e = zeros(1,2) ;
        
        inch2m = 0.0254;
        kin = HebiKinematics();
        kin.addBody('FieldableElbowJoint');
        kin.addBody('FieldableElbowJoint');
        kin.addBody('FieldableElbowLink', ...
            'ext1', 4 * inch2m, 'twist1', pi/2, ...
            'ext2', 0.5 * inch2m, 'twist2', pi);
        kin.addBody('FieldableElbowJoint');
        kin.addBody('FieldableStraightLink', ...
            'ext', 6 * inch2m, 'twist', -pi/2);
        kin.addBody('FieldableElbowJoint');
        kin.addBody('FieldableGripper');
        jointLocations = [1, 2, 4, 6, 7];
        loghebi = HebiUtils.newGroupFromLog('log1.hebilog');
        robot = Robot(log.position(1,:), 0, kin, jointLocations);
        robot.NumberOfJoints = joints;
        robot.Gyros.NumberOfGyros = joints;
    else
        e = zeros(1,2);    
        samples = [100, 100, 100, 100, 100, 100];%180;
        ssamples = sum(samples);

        q_f = zeros(ssamples, joints);
        t_ipo = ones(1, ssamples)*0.02;
        multipliers = [0.05 0.1 0.2 0.4 0.65 1];
        for i=1:joints
            x = linspace(0, 2*pi, samples(i));
            y = rad2deg(sin(x)*multipliers(i));
            if i == 1
                start = 1;
            else
                start = sum(samples(1:i-1))+1;
            end
            last = sum(samples(1:i));
            q_f(start:last,i) = y;
        end
    end
end
q_i = 2;

for seg = 2:length(e)
    
    if angleControl == 0
        e1 = e{seg-1}(1:6);
        e2 = e{seg}(1:6);
        v = 2*vc;

        [tx ax] = robot.create_lin_seg_list(e1,e2,v,amax,t_ipo(1));
        [t, a, v, s] = robot.create_lin_intvec(tx, ax, t_ipo(1));
        ec = robot.create_lin_path(e1,e2,s);

        maxIter = length(ec);
        t_ipo = [t_ipo ones(1, maxIter)*t_ipo(1)];
    elseif angleControl == 1
        maxIter = length(t_ipo)-1;
    end
    for i=1:maxIter
        if i == 1079
            aaa = 1;
        end
        if angleControl == 0
            tg = xyzabc_2_t(ec{i}(1), ec{i}(2), ec{i}(3), ec{i}(4), ec{i}(5), ec{i}(6));
            q(:,q_i) = robot.kr16_rk(tg,robot.Base,robot.Endeffector,pose);         
        elseif angleControl == 1
            if i == 1
                q(:, 1) = q_f(1,:)';
            end 
            q(:, q_i) = q_f(q_i,:)'; 
            
        end
        
        q_d = q(:,q_i) - q(:,q_i-1);

        if useLogFile == 1
            Tsubt = kin.getForwardKinematics('output', q(:,q_i));
            for i=1:joints
                Tsub{i} = Tsubt(:, :, jointLocations(i));
            end
            robot.CalculateGyroPosition(Tsub);
            jointVelocities(q_i,:) = robot.CalculateJointVelocities(q_d, t_ipo(q_i));
            robot.CalculateAngularVelocities(t_ipo(q_i)); % t_ipo(q_i)
        else
            robot.Time(q_i) = robot.Time(q_i-1) + t_ipo(q_i);    % t_ipo(q_i)
            robot.JointPositions(q(:,q_i));
            [T Tsub rot] = robot.fk_craig(q, robot);
            robot.CalculateGyroPosition(Tsub);
            jointVelocities(q_i,:) = robot.CalculateJointVelocities(q_d, t_ipo(q_i));   % t_ipo(q_i)
            robot.CalculateAngularVelocities(t_ipo(q_i)); % t_ipo(q_i)
        end
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
% axis([-2000 2000 -2000 2000 -0.2 2000]);
% view([102,20]);
% view(90,0);

grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');

% draw_robot_path(q,t_ipo,robot,0.1, 1, robot.Gyros.Positions, kin);
% cc2
if useLogFile == 1
         links = {{'FieldableElbowJoint'},
                {'FieldableElbowJoint'},
                {'FieldableElbowLink', ...
                    'ext1', 4 * inch2m, 'twist1', pi/2, ...
                    'ext2', 0.5 * inch2m, 'twist2', pi},
                {'FieldableElbowJoint'},
                {'FieldableStraightLink', ...
                    'ext', 6 * inch2m, 'twist', -pi/2};   
                 {'FieldableElbowJoint'},
                 {'FieldableElbowJoint'}};
     plt = HebiPlotter('JointTypes', links);
     q = q';
     for i=1:size(q,1)
            plt.plot(q(i,:));
            
            
            key = get(gcf,'CurrentKey');
            if(strcmp (key , 'return'))
                break;
            end
    
    end
else
%     axis([-10 10 -10 10 -10 10]);
     axis([-2000 2000 -2000 2000 -0.2 2000]);
    draw_robot_path(q,t_ipo,robot,0.1, 1, robot.Gyros.Positions);
    
end

asd = 1
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
% gyros{1} = robot.Gyros.OmegasX;
% gyros{2} = robot.Gyros.OmegasY;
% gyros{3} = robot.Gyros.OmegasZ;
% log.gyroX = robot.Gyros.OmegasX;
% log.gyroY = robot.Gyros.OmegasY;
% log.gyroZ = robot.Gyros.OmegasZ;
% log.velocity = jointVelocities;
% log.time = robot.Time;
% log.position = robot.Joints.PositionsRad;
% log.numModules = joints;

gyros{1} = log.gyroX;
gyros{2} = log.gyroY;
gyros{3} = log.gyroZ;
gyrosTitle = {'Gyro X', 'Gyro Y', 'Gyro Z'};

figure
for i=1:3
    subplot(3,1,i)
    hold on;
%     for j=1:joints
% %         if j == 4
% %             plot(gyros{i}(:,j));    
% %         elseif j == 3
% %             plot(gyros{i}(:,j), 'o');
% %         else
%             plot(gyros{i}(:,j));
% %         end
%     end
    plot(gyros{i})
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



% p = robot.Gyros.Positions;
% figure
% hold on
% for l=1:size(robot.Gyros.Positions,1)
%  p = robot.Gyros.Positions(l, :);
%  cla
%  hold on
%     for i=1:size(robot.Gyros.Positions,2)
% 
%         drawingMode = {'bo', 'go', 'ro'};
%         for j=1:3
% 
%             plot3(p{i}{j}{1}(1, 4), p{i}{j}{1}(2, 4), p{i}{j}{1}(3, 4), drawingMode{j});
%             plot3(p{i}{j}{2}(1, 4), p{i}{j}{2}(2, 4), p{i}{j}{2}(3, 4), drawingMode{j});
%             plot3(p{i}{j}{3}(1, 4), p{i}{j}{3}(2, 4), p{i}{j}{3}(3, 4), drawingMode{j});
%         end
%     end
% end


%jointDetectionDave(log);
jointDetectionSingle(log);
jointDetectionDave(log);
