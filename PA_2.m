%% Good Matlab Practices
clc; clear; close all;

%% End Effector Zero Position Configuration
L=0.5; %m (given)
M=[0 0 1 L ; 0 -1 0 0 ; 1 0 0 0 ; 0 0 0 1];

%% Matrix Representation of Screw Motion for Each Joint [Si]
S1=[0 -1 0 0;1 0 0 0;0 0 0 0;0 0 0 0];
S2=[0 0 -1 0;0 0 0 0;1 0 0 0;0 0 0 0];
S3=[0 0 -1 0;0 0 0 0;1 0 0 -L;0 0 0 0];
S4=[0 0 0 1;0 0 0 0;0 0 0 0;0 0 0 0];

% %% Introduction
% fprintf('Hello, this program finds the end effector final configuration given user inputs for a RRRP system. \nPress spacebar to continue.\n\n')
% pause
% 
% %% Ask for User Input and Error Check
% start=1;  %Start
% error=0;  %No Error at start
% 
% %theta1 input
% while error==1 || start==1    %While error is true or it is the first iteration
%     theta1=input('Please enter the joint parameter for the revolute joint 1, in radians\n');
%     if theta1>=0 || theta1<0
%         break  
%     else
%         fprintf('Invalid entry, please try again.\n')
%         error=1;
%     end
% end
% 
% %theta2 input
% while error==1 || start==1    %While error is true or it is the first iteration
%     theta2=input('Please enter the joint parameter for the revolute joint 2, in radians\n');
%     if theta2>=0 || theta2<0
%         break  
%     else
%         fprintf('Invalid entry, please try again.\n')
%         error=1;
%     end
% end
% 
% %theta3 input
% while error==1 || start==1    %While error is true or it is the first iteration
%     theta3=input('Please enter the joint parameter for the revolute joint 3, in radians\n');
%     if theta3>=0 || theta3<0
%         break  
%     else
%         fprintf('Invalid entry, please try again.\n')
%         error=1;
%     end
% end
% 
% %theta4 input
% while error==1 || start==1    %While error is true or it is the first iteration
%     theta4=input('Please enter the joint parameter for the prismatic joint 4, in meters. Note: Must be positive\n');
%     if theta4>=0
%         break  
%     else
%         fprintf('Invalid entry, please try again.\n')
%         error=1;
%     end
% end


%% Overwritten 
theta1 = pi/3
theta2 = pi/2
theta3 = pi/4
theta4 = 1.2
%% FK Expression: PoE Function and End Effector Position and Orientation Matrix
EEPO=expm(S1*theta1)*expm(S2*theta2)*expm(S3*theta3)*expm(S4*theta4)*M;

%% Yaw-Pitch-Roll: ZYX Euler Angles, Section B.1.1 in Book
beta=atan2(-EEPO(3,1),sqrt(EEPO(1,1)^2+EEPO(2,1)^2));  %Revolution around y-axis
alpha=atan2(EEPO(2,1),EEPO(1,1));   %Revolution around z-axis
gamma=atan2(EEPO(3,2),EEPO(3,3));   %%Revolution around x-axis

%% Print out of End Effector Position and Orientation, Including Yaw-Pitch-Roll angles (ZYX)
fprintf('\nThe roll angle for the end-effector is %f rad\n', gamma)
fprintf('The pitch angle for the end-effector is %f rad\n', beta)
fprintf('The yaw angle for the end-effector is %f rad\n', alpha)

fprintf('\nThe coordinate of the end-effector is %f , %f ,%f',EEPO(1,4), EEPO(2,4), EEPO(3,4))
fprintf('\nThe end-effector orientation and position matrix is:')
EEPO
%% Simulation
t = 0;

dtheta1 = theta1/150;
dtheta2 = theta2/150;
dtheta3 = theta3/150;
dtheta4 = theta4/150;

for t = 0:150;
    EEPO=expm(S1*dtheta1*t)*expm(S2*dtheta2*t)*expm(S3*dtheta3*t)*expm(S4*dtheta4*t)*M;
    EEP3=expm(S1*dtheta1*t)*expm(S2*dtheta2*t)*expm(S3*dtheta3*t)*M;;
    
    x=[0 0 EEP3(1,4) EEPO(1,4)];
    y=[0 0 EEP3(2,4) EEPO(2,4)];
    z=[-0.3 0 EEP3(3,4) EEPO(3,4)];%-0.3 is used to seperate joint 1 and joint 2, coordinate of joint 1 is [0 0 -0.3
    
    
    scatter3(x,y,z);
    line(x,y,z);
    xlim([-1 1]);
    ylim([-1 1]);
    zlim([-0.3 2]);
    grid on;
    
    F = getframe;
end

movie(F);
    
    


