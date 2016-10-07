
load Ahead.mat

ms=5;

alt=vect(1:3:end-2);
vx=vect(2:3:end-1);
vy=vect(3:3:end);

n=length(vx);

t=0:5:5*(n-1);

plot3(vx,vy,alt)
xlabel('vx')
ylabel('vy')
zlabel('alt')
grid on
% plot(t,alt)