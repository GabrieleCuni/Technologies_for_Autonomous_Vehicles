close all
clear all
clc

x_h = 0;
v_h = 28;
x_l = 50;
v_l = 36;
v_tar = 36;
t_h_tar = 1;
d_0 = 8;
Kv = 0.5;
Kd_err = 0.2;
Kv_r = 0.4;
open('host_leading.slx');
sim('host_leading.slx');

%% check if host vehicle touches leading vehicle
time = size(ans.tout,1);
for i=1:time
    if ans.x_l(i) <= ans.x_h(i)
        return;
    end
end


%% little animation
figure(1);
for i=1:10:time
    plot(ans.x_h(i), 10, 'marker', 's');
    axis([-5 2100 8 12]);
    hold on;
    plot(ans.x_l(i), 10, 'marker', 's');
    hold off;
    pause(0.1);
end