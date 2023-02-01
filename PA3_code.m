%% Good Matlab Practices
clc; clear; close all;

%% Given Values
L1=1;
L2=0.5;

%% Creating Plot
%Inner Circle Bound
r1=L1-L2;
x0=0;
y0=0;
th=0:0.001:2*pi;
xvalue1=r1*cos(th)+x0;
yvalue1=r1*sin(th)+y0;
plot(xvalue1,yvalue1,'r');
xlim([-2 2])
ylim([-2 2])
hold on
 
%Outer Circle Bound
r2=L1+L2;
xvalue2=r2*cos(th)+x0;
yvalue2=r2*sin(th)+y0;
plot(xvalue2,yvalue2,'b');
xlabel('X-Position')
ylabel('Y-Position')
title('2R Linkage Path Tracking')
 
%% User Inputs
n = input("How many points you want to select?\n")

if n <2 || ~(floor(n)==n) || n == inf || n == -inf
    error('please enter valid integer!')
    return;
end

for i=1:n    
    clicksLeft=n+1-i;
    s=sprintf('Pick %i more points',clicksLeft);
    changingText=text(0.75,1.5,s);
    [x(i),y(i)]=ginput(1);
    plot(x,y,'--*k');
    delete(changingText)
end

%Results in vectors of chosen points: x vector and y vector positions



%% Inverse Kinematics
% This section of code removes the linear splines between points
pause
hold off
plot(x,y,'*k')
hold on
plot(xvalue1,yvalue1,'r');
plot(xvalue2,yvalue2,'b');
xlim([-2 2])
ylim([-2 2])
xlabel('X-Position')
ylabel('Y-Position')
title('2R Linkage Path Tracking')

% Now plot path and create linkages that follow the path, except when path
% doesn't stay within the bounds, then follow the edge of the bounds

L_1 = L1;
L_2 = L2;

X = x;
Y = y;

frame_rate = 30;
dframe = 0.8;

theta1 = zeros(1,n);
theta2 = zeros(1,n);

for i = 1:n
    theta1(i) = atan2(Y(i),X(i))-acos((X(i)^2+Y(i)^2+L_1^2-L_2^2)/(2*L_1*sqrt(X(i)^2+Y(i)^2)));
    theta2(i) = acos((L_1^2+L_2^2-X(i)^2-Y(i)^2)/(-2*L_1*L_2));
end
%% Movement Generation
theta1_tr = theta1;
theta2_tr = theta2;


t = zeros(4,n);

for k = 2: n
    t(1,k) = abs(theta1(k)-theta1(k-1));
    t(2,k) = abs(theta2(k)-theta2(k-1));
    t(3,k) = max(t(:,k));
    t(4,k) = t(3,k)+t(4,k-1);
end
total_time = sum(t(3,:))*dframe;

total_frame = round(total_time*frame_rate+0.5);

theta1_t = zeros(2,n);
theta1_t(1,:) = theta1;

theta2_t = zeros(2,n);
theta2_t(1,:) = theta2;

for j = 1:n
    theta1_t(2,j) = round(dframe*t(4,j)/total_time*total_frame);
    theta2_t(2,j) = round(dframe*t(4,j)/total_time*total_frame);
end

if n >=4
    theta1_m= interp1(theta1_t(2,:),theta1_t(1,:),1:total_frame,'spline');
    theta2_m= interp1(theta2_t(2,:),theta2_t(1,:),1:total_frame,'spline');
else
    theta1_m= interp1(theta1_t(2,:),theta1_t(1,:),1:total_frame,'linear');
    theta2_m= interp1(theta2_t(2,:),theta2_t(1,:),1:total_frame,'linear');
end


nnn=0;
for i = 1:length(theta1_m)
if isreal(theta1_m(i))
nnn = nnn+1;
end
end

joint1 = zeros(2,total_frame);
joint2 = zeros(2,total_frame);

for i = 1: total_frame
    joint1(1,i) = L_1*cos(theta1_m(i));
    joint1(2,i) = L_1*sin(theta1_m(i));
    joint2(1,i) = L_1*cos(theta1_m(i))+L_2*cos(theta1_m(i)+theta2_m(i));
    joint2(2,i) = L_1*sin(theta1_m(i))+L_2*sin(theta1_m(i)+theta2_m(i));
end
%% Ploting
hold off;
for t = 1: total_frame
    plot(x,y,'*k')
    hold on
    plot(xvalue1,yvalue1,'r');
    plot(xvalue2,yvalue2,'b');
    xlim([-2 2])
    ylim([-2 2])
    xlabel('X-Position')
    ylabel('Y-Position')
    title('2R Linkage Path Tracking')
    plot(joint2(1,:),joint2(2,:));
    a = [x0 joint1(1,t) joint2(1,t)];
    b = [y0 joint1(2,t) joint2(2,t)];
    scatter(a,b)
    line(a,b)
    
    F = getframe;
    hold off;
end

movie(F,1,frame_rate)