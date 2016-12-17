
load Same_orientation_4.mat
%load UGV_straight_line_2_cmd.mat

ms=5;

alt=vect(1:3:end-2);
% Get velocity in mm/s
vx=vect(2:3:end-1)/10;
vy=vect(3:3:end)/10;

ugv_vel = ugv_vel(1:end);

n=length(vx);

t=0:1:1*(n-1);  % Create time vector of 5ms per interval (sampling time)

% Plot velocity
%plot3(vx,vy,alt)
%xlabel('vx')
%ylabel('vy')
%zlabel('alt')
%grid on
% plot(t,alt)

% Calculate displacement integrating velocities in millimeters
displacement_x = cumtrapz(t, vx);
displacement_y = cumtrapz(t, vy);

% Convert displacements in meter
displacement_x = displacement_x/10000;
displacement_y = displacement_y/1000;
alt = alt/1000;

%--------------------------------------------------------------------------
% Plot trajectory
f1 = figure('Name', 'Trajectory')
subplot(2,1,1)
p11 = plot3(displacement_x, - displacement_y, alt) 
grid on
title('Quadcopter trajectory in space');
xlabel('Displacement X');
ylabel('Displacement Y');
zlabel('Altitude');

subplot(2,1,2)
p12 = plot(-displacement_y, - displacement_x)
grid on;
title('Planar trajectory');
xlabel('Displacement Y');
ylabel('Displacement x');

%--------------------------------------------------------------------------
% Convert time in seconds and velocity in mm/s
t = t/100;
vx = vx/10;
vy = vy/10;

% Plot velocities
f2 = figure('Name','Quadcopter Graphs')
p21 = subplot(5,1,1)
plot(t, vx)
grid on;
title('Quadcopter');
xlabel('Time (s)');
ylabel('Velocity Vx (m/s) ');

subplot(5,1,2)
p22 = plot(t, vy)
grid on;
% title('Quadcopter linear velocity (y)');
xlabel('Time (s)');
ylabel('Velocity Vx (m/s) ');

subplot(5,1,3)
p23 = plot(t, displacement_x)
grid on;
%title('Quadcopter displacement (x)');
xlabel('Time (s)');
ylabel('Displacement on X (m) ');

subplot(5,1,4)
p24 = plot(t, - displacement_y)
grid on
% title('Quadcopter displacement (y)');
xlabel('Time (s)');
ylabel('Displacement on Y (m) ');

subplot(5,1,5)
p24 = plot(t, alt)
grid on
%title('Quadcopter altitude');
xlabel('Time (s)');
ylabel('Altitude (m) ');

%--------------------------------------------------------------------------
% f3 = figure('Name','UGV Graphs')
% p31 = plot(t, ugv_vel)
% ylim([0 0.3])
% grid on
% title('UGV');
% xlabel('Time (s)');
% ylabel('Velocity Vx (m/s)');