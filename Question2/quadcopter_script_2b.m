%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%  Visualisation code for quadcopter 
%%%%  Author: Xiang Long
%%%%  Date: 2023/10/17
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;
close all;

%Define total width, length and height of flight arena (metres)
spaceDim = 10;
spaceLimits = [-spaceDim/2 spaceDim/2+5 -spaceDim/2-5 spaceDim/2 0 spaceDim/2+5];

%do you want to draw a ground image on the figure?
draw_ground = true;
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
    drones = [drones Drone_2b(ax1, spaceDim, num_drones)];
end

while(drones(1).time < 8.0)
    %clear axis
    cla(ax1);
    
    %update and draw drones
    for i = 1:num_drones
        update(drones(i));
    end
    
    %optionally draw the ground image
    if(draw_ground)
        imagesc([-spaceDim,spaceDim],[-spaceDim,spaceDim],ground_img);
    end
    
    drone_trajectory=[
    drone_trajectory drones(1).pos(1:3) ];
    drone_orientation = [
        drone_orientation drones(1).y(7:9)];
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
    %apply fancy lighting (optional)
    camlight
    
    %update figure
    drawnow
    pause(0.01)
end

subplot(3,2,1)
plot(t,x);
title("X position");

subplot(3,2,2)
plot(t,roll);
title("roll angle");

subplot(3,2,3)
plot(t,y);
title("Y position");

subplot(3,2,4)
plot(t,pitch);
title("Pitch angle");

subplot(3,2,5)
plot(t,z);
title("Z position");

subplot(3,2,6)
plot(t,yaw);
title("Yaw angle");