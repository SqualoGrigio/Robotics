clc;
clear;

theta1_0 = -pi/4;
theta2_0 = -pi/4;
dtheta1_0 = 0;
dtheta2_0 = 0;

param.m1 = 0.2; param.m2 = 0.2;
param.L1 = 0.25; param.L2 = 0.25;
param.I1 = 1/12; param.I2 = 1/12;
param.r1 = param.L1; param.r2 = param.L2;
param.g = 9.81;

param.kp1 = 51.18; param.kd1 = 28.59;
param.kp2 = 16; param.kd2 = 4;

L_1 = param.L1;
L_2 = param.L2;

x0=0;
y0=0;
th=0:0.001:2*pi;
xvalue1=0.5*cos(th)+x0;
yvalue1=0.5*sin(th)+y0;
plot(xvalue1,yvalue1,'r');
xlim([-0.6 0.6])
ylim([-0.6 0.6])
xlabel('X-Position')
ylabel('Y-Position')
hold on;

[x,y]=ginput(1);
k = x*x+y*y;
if k>0.25
    error('please click a valid point!')
end

plot(x,y,'--*k');
hold off;

param.theta1_d = atan2(y,x)-acos((x^2+y^2+L_1^2-L_2^2)/(2*L_1*sqrt(x^2+y^2)));
param.theta2_d = acos((L_1^2+L_2^2-x^2-y^2)/(-2*L_1*L_2));

frame = 1200;
frame_time = 12;
tspan = linspace(0,frame_time,frame); % simulation duration
statevar0 = [theta1_0; dtheta1_0; theta2_0; dtheta2_0]; % initial conditions 
options = odeset('reltol',1e-9,'abstol',1e-9); % specifies accuracy of simulation
[tlist1,statevarlist1] = ode45(@vdp3,tspan,statevar0,options,param);

theta1list = statevarlist1(:,1);
dtheta1list = statevarlist1(:,2);
theta2list = statevarlist1(:,3);
dtheta2list = statevarlist1(:,4);


figure(2);
plot(tlist1,theta1list);
ylabel('theta_1');
xlabel('t');
figure(3);
plot(tlist1,theta2list);
ylabel('theta_2');
xlabel('t');

figure(4);
plot(tlist1,dtheta1list);
ylabel('dtheta_1');
xlabel('t');
figure(5);
plot(tlist1,dtheta2list);
ylabel('dtheta_2');
xlabel('t');

figure(1);

joint1tx = zeros(frame,1);
joint1ty = zeros(frame,1);
joint2tx = zeros(frame,1);
joint2ty = zeros(frame,1);

for i = 1:frame
    joint1tx(i) = L_1*cos(theta1list(i));
    joint1ty(i) = L_1*sin(theta1list(i));
    joint2tx(i) = L_1*cos(theta1list(i))+L_2*cos(theta1list(i)+theta2list(i));
    joint2ty(i) = L_1*sin(theta1list(i))+L_2*sin(theta1list(i)+theta2list(i));
end

for i = 1:1200
    x0=0;
    y0=0;
    th=0:0.001:2*pi;
    xvalue1=0.5*cos(th)+x0;
    yvalue1=0.5*sin(th)+y0;
    plot(xvalue1,yvalue1,'r');
    hold on;
    xlim([-0.6 0.6]);
    ylim([-0.6 0.6]);
    xlabel('X-Position');
    ylabel('Y-Position');
    plot(x,y,'--*k');
    a = [0 joint1tx(i) joint2tx(i)];
    b = [0 joint1ty(i) joint2ty(i)];
    scatter(a,b);
    line(a,b);
    xlim([-0.6 0.6]);
    ylim([-0.6 0.6]);
    time = i /120;
    txt = ['t = ' num2str(time)];
    text(-0.5,0.5,txt);
    F = getframe;
    hold off;
end

movie(F,1,30);




