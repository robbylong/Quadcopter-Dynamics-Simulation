%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%  Visualisation code for quadcopter 
%%%%  Author: Xiang Long
%%%%  Date: 2023/10/17
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Define total width, length and height of flight arena (metres)
clear all;
close all;

spaceDim = 20;
spaceLimits = [-spaceDim/2 spaceDim/2 -spaceDim/2 spaceDim/2 0 spaceDim];

%do you want to draw a ground image on the figure?
draw_ground = false;
if(draw_ground)
    ground_img = imread('ground.png');
end


%figure to display drone simulation
f1 = figure;
ax1 = gca;
view(ax1, 3);
axis equal;
axis(spaceLimits)
grid ON
grid MINOR
caxis(ax1, [0 spaceDim]);
hold(ax1,'on')
axis vis3d



num_drones = 1;

%instantiate a drone object, input the axis and arena limits
drones = [];
drone_trajectory = [];
drone_orientation = [];
drone_time = [];

for i = 1:num_drones
    drones = [drones Drone_3a(ax1, spaceDim, num_drones,[0;0;0])];
end

%calculate circular path
centre = [2.5;5;5];
angle_interval = -pi/6;
num_intermediate_point = 2;
num_points = 4*num_intermediate_point+4;
path = zeros(3,num_points);
path(1,1) = 5;
path(2,1) = 5;
path(3,1) = 5;
radius = 2.5;

stage = 0;

constant_speed = 0.5;

%set eigenvalue
e1 = 0.9.*[1,1,1];
e2 = 0.93.*[1,1,1];
e3 = 0.95.*[1,1,1];
e4 = 0.94.*[1,1,1];
eigenvalues = [e1,e2,e3,e4];

data=load("disc_matrices.mat");
K = place(data.sys_d.A,data.sys_d.B,eigenvalues);

for i = 1:num_points
    path(1,i+1) = centre(1)+radius*cos(i*angle_interval);
    path(2,i+1) = centre(2)+radius*sin(i*angle_interval);
    path(3,i+1) = 5;
end
while(drones(1).time < 50.0)
    %clear axis
    cla(ax1);


    switch stage
        case 0         
            rx = [5;5;5;0;0;0;0;0;0;0;0;0];
            d_e=rx(1:3)-drones(1).pos;
            v_norm=norm(d_e);
            rx=[5;5;5;d_e(1)/v_norm*constant_speed;d_e(2)/v_norm*constant_speed;d_e(3)/v_norm*constant_speed;0;0;0;0;0;0];
            e = [drones(1).pos;drones(1).xdot;drones(1).theta;drones(1).omega]-rx;
            pos = drones(1).pos
            if norm(e)<0.7
                start_lifting_t = drones(1).time;
                stage = 1;
                disp('drone has arrived target point (5,5,5), will be staying for 5 seconds')
            end


        case 1
            count = drones(1).time-start_lifting_t
            if (drones(1).time>start_lifting_t+1)
            % if (drones(1).time>start_lifting_t+5)
                stage = 2;
                start_circle_t = drones(1).time;
                disp('now the drone will move in a circular path')
            end
           
        case 2
 
            X=[drones(1).pos;drones(1).xdot;drones(1).theta;drones(1).omega];

            theta=-atan2(X(2)-5,X(1)-2.5);

            rx=[2.5+2.5*cos(theta);5-2.5*sin(theta);5;-constant_speed*sin(theta);-constant_speed*cos(theta);0;0;0;0;0;0;0];
%             ref(1:6)
            %X(1:6)
            
            pos = drones(1).pos

            if (abs(theta)<0.05)&&(drones(1).time>start_circle_t+2)
                stage = 3;
                disp('now the drone will land slowly')
                total_dis = norm([5;5;0]-[5;5;5]);
            end
        
        case 3
            X = [drones(1).pos;drones(1).xdot;drones(1).theta;drones(1).omega];
            
            cur_dis = [5;5;0]-X(1:3);
            cur_vx = constant_speed*cur_dis(1)*cur_dis(1)/total_dis;   
            cur_vy = constant_speed*cur_dis(2)*cur_dis(2)/total_dis;   
            cur_vz = constant_speed*cur_dis(3)*cur_dis(3)/total_dis;   
            rx = [5;5;0;cur_vx;cur_vy;cur_vz;0;0;0;0;0;0];
            pos_and_v = [X(1:3) X(4:6)]
            if norm([5;5;0]-X(1:3))<0.05
                stage = 4
                linear_velocity = norm(X(4:6))
                disp('the drone has landed successfully!')
            end

    end
    % add gaussian noise to sensor measurements
    e = [drones(1).pos;drones(1).xdot;drones(1).theta;drones(1).omega]+[randn(3,1)*0.2;randn(3,1)*0.2;randn(3,1)*0.03;randn(3,1)*0.03]-rx;
    drones(1).u = -K*e;
    for i=1:1:4
        if drones(1).u(i)<-5e2
            drones(1).u(i) = -5e2;
        end
        if drones(1).u(i)>5e2
            drones(1).u(i) = 5e2;
        end
    end
    drones(1).u = drones(1).u + 0.49;
    
    drone_trajectory=[
        drone_trajectory drones(1).pos(1:3)];
    drone_orientation = [
        drone_orientation drones(1).theta(1:3)];
    drone_time = [
        drone_time drones(1).time];
    x = drone_trajectory(1,1:width(drone_trajectory));
    y = drone_trajectory(2,1:width(drone_trajectory));
    z = drone_trajectory(3,1:width(drone_trajectory));
    roll = drone_orientation(1,1:width(drone_orientation));
    pitch = drone_orientation(2,1:width(drone_orientation));
    yaw = drone_orientation(3,1:width(drone_orientation));
    t = drone_time(1:width(drone_time));
    plot3(ax1,x,y,z,'b')
    %update and draw drones
    for i = 1:num_drones
        update(drones(i));
    end
    
    %optionally draw the ground image
    if(draw_ground)
        imagesc([-spaceDim,spaceDim],[-spaceDim,spaceDim],ground_img);
    end
    
    %apply fancy lighting (optional)
    camlight
    
    %update figure
    drawnow
    pause(0.01)
end

subplot(3,2,1)
plot(t,x);
ylabel('X(m)');
xlim([0 47]);
title("X position");
grid on;

subplot(3,2,2)
plot(t,roll*180/pi);
xlim([0 47]);
ylabel('roll(degree)');
title("roll angle");
grid on;

subplot(3,2,3)
plot(t,y);
xlim([0 47]);
ylabel('Y(m)');
title("Y position");
grid on;

subplot(3,2,4)
plot(t,pitch*180/pi);
ylabel('pitch(degree)');
xlim([0 47]);
title("Pitch angle");
grid on;

subplot(3,2,5)
plot(t,z);
ylabel('z(m)');
xlim([0 47]);
title("Z position");
grid on;

subplot(3,2,6)
plot(t,yaw*180/pi);
ylabel('yaw(degree)');
xlim([0 47]);
title("Yaw angle");
grid on;
