
close all
clear all
    
% Assignment belt variable values
m=20;    % Mass of the sled [kg]
M=m;
a=1;     % m/s2
b=40;    % Sled friction coefficient [N/(m/s)] (Friction force of the sled = b*(sled speed))
k=1e6;   % Spring constant of the belt [N/m]

% Uncomment if these are needed.
% bs=40
% br=0.5


% Motor variable values
T_N=2.0;   % Nominal torque of the motor [Nm]
rpmN=2400; % Nominal speed of the motor [rpm]
J=0.0028;  % JSUURI (Comment out the J below if you want to test the system with a bigger J value.)
J=1.08e-4; % Inertia of the motor [kgm^2]

r=0.032; % Radius of the pulley [m]

% T/r=F
% Js=J/gr^2
% Js=J; % no gear J

%%  state space

% For system given in EX. work
% No roller friction, mass friction exists , no damping of belt 

% States: 
%   x1:rotorAngle 
%   x2:rotor angular frequency
%   x3:position of cart 
%   x4:velocity of cart

% Notice that one state less would do , we have difference k(r*theta-x) 
% leading to states e.g. x1=r*theta-x, x2=r*d(theta)/dt, and x3=dx/dt
% first we take these as separate states, no harm done but sometimes
% diffuculties in analysis with matlab, simulation goes well, and gives
% reliable background for analysis.
% See model page 38, Lecture1_mechmodels.pdf, Week3 in Moodle.


A = [      0       1        0        0; 
      -(k*r^2)/J   0     (k*r)/J     0; 
           0       0        0        1; 
        (k*r)/M    0      -k/M     -b/M];

B = [ 0; 
     1/J; 
      0; 
      0];  
  
C=[ 0 0 1 0;
    0 0 0 1;
    0 1 0 0 ];

D=[0;0;0];

sys_ss=ss(A,B,C,D);

eig(A)
% figure(1);
% step(sys_ss) % Draws step responses in subplots of the same figure.

%% cart position

C_pos=[0 0 1 0];step(sys_ss)
D_pos=[0];
sys_ss_pos=ss(A,B,C_pos,D_pos);

figure(2);
step(sys_ss_pos,'r')
title('Cart position step response')
xlim([0 4.5]) % Same xlim as with speed

figure(3);
impulse(sys_ss_pos,'r')
title('Cart position impulse response')

figure(4);
margin(sys_ss_pos,'r')

%% cart speed

C_speed=[0 0 0 1];
D_speed=[0];
sys_ss_speed=ss(A,B,C_speed,D_speed);

figure(5);
step(sys_ss_speed,'b')
title('Cart speed step response')
xlim([0 4.5])

figure(6);
impulse(sys_ss_speed,'b')
title('Cart speed impulse response')

figure(7);
margin(sys_ss_speed,'b')