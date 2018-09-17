close all; clear; clc;

filename = 'highway_map.csv';
map = dlmread(filename, ' ');

x = map(:,1);
y = map(:,2);
s = map(:,3);
dx = map(:,4);
dy = map(:,5);

figure; hold on;
plot(x,y,'b--x');
axis equal;

lane_width = 4;
for lane_id = 0:2
  lane_x = x + (lane_id + 1) * lane_width * dx;
  lane_y = y + (lane_id + 1) * lane_width * dy;
  plot(lane_x,lane_y,'b--x');
end

filename = 'highway_map_high_precision.csv';
map = dlmread(filename, ' ');
for i = 0:3
  plot(map(:,2*i+1),map(:,2*i+2),'r-');
end