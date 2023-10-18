clear all;
close all;
data1 = load('question3a_variables.mat');
data2 = load('question3b_variables.mat');
figure
subplot(3,2,1)
plot(data1.t,data1.x);
hold on;
plot(data2.t,data2.x);
xlim([0 50]);
title("X position");
grid on;

subplot(3,2,2)
plot(data1.t,data1.roll*180/pi);
hold on;
plot(data2.t,data2.roll*180/pi);
xlim([0 50]);
title("roll angle");
grid on;

subplot(3,2,3)
plot(data1.t,data1.y);
hold on;
plot(data2.t,data2.y);
xlim([0 50]);
title("Y position");
grid on;

subplot(3,2,4)
plot(data1.t,data1.pitch*180/pi);
hold on;
plot(data2.t,data2.pitch*180/pi);
xlim([0 50]);
title("Pitch angle");
grid on;

subplot(3,2,5)
plot(data1.t,data1.z);
hold on;
plot(data2.t,data2.z);
xlim([0 50]);
title("Z position");
grid on;

subplot(3,2,6)
plot(data1.t,data1.yaw*180/pi);
hold on;
plot(data2.t,data2.yaw*180/pi);
xlim([0 50]);
legend('with zero noise','with Gaussian noise');
title("Yaw angle");
grid on;