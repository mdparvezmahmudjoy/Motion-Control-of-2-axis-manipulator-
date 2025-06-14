clc;
clear all; 
close all;

% I got to check this right before the identification, that's where I'll
% continue tomorrow/today. (or maybe setting controller values first)

% The PI controller I part was formed from I=1/Ti here but Simulink had the
% value as I=Ti. (I=1/Ti is the correct presentation form, so if the
% Simulink model PI controller input is changed to that then those two should )

% I changed the open loop systems which the sensitivity functions used.
% (Cascade form makes them not so simple, I made a figure of it in my notes)


%% Assignment part I

% Assignment belt variable values
m=20;    % Mass of the sled [kg]
M=m;
a=1;     % m/s2
b=40;    % Sled friction coefficient [N/(m/s)] (Friction force of the sled = b*(sled speed))
k=1e6;   % Spring constant of the belt [N/m]

% Uncomment if these are needed.
% bs=40
% br=0.5


Ts_pseudo = 1;

% Motor variable values
T_N=2.0;   % Nominal torque of the motor [Nm]
rpmN=2400; % Nominal speed of the motor [rpm]
J=0.0028;  % JSUURI (Comment out the J below if you want to test the system with a bigger J value.)
J=1.08e-4; % Inertia of the motor [kgm^2]

r=0.032; % Radius of the pulley [m]

% T/r=F
% Js=J/gr^2
% Js=J; % no gear J

%%  State space

% For system given in EX. work
% No roller friction, mass friction exists , no damping of belt 

% States: 
%   x1:rotorAngle 
%   x2:rotor angular frequency
%   x3:position of cart 
%   x4:velocity of cart

A = [      0       1        0        0; 
      -(k*r^2)/J   0     (k*r)/J     0; 
           0       0        0        1; 
        (k*r)/M    0      -k/M     -b/M];

B = [ 0; 
     1/J; 
      0; 
      0];  
  
C=[ 0 0 1 0;
    0 0 0 1];

D=[0;0];

sys_ss=ss(A,B,C,D);


eig(A)
% figure
% step(sys_ss) % Draws step responses in subplots of the same figure.

%% Position, open loop

C_pos=[0 0 1 0];step(sys_ss)
D_pos=[0];
sys_ss_pos=ss(A,B,C_pos,D_pos);

figure
step(sys_ss_pos,'r')
title('Position step response')
xlim([0 4.5]) % Same xlim as with speed

figure
impulse(sys_ss_pos,'r')
title('Position impulse response')

figure
margin(sys_ss_pos,'r')

%% Speed, open loop

C_speed=[0 0 0 1];
D_speed=[0];
sys_ss_speed=ss(A,B,C_speed,D_speed);

figure
step(sys_ss_speed,'b')
title('Speed step response')
xlim([0 4.5])

figure
impulse(sys_ss_speed,'b')
title('Speed impulse response')

figure
margin(sys_ss_speed,'b')




[num, den] = tfdata(sys_ss)
sys2_pos = tf(num(1),den(1))
sys2_speed = tf(num(2),den(2))



%% Speed & Pos PI-controller:

% Speed controller values
K_p_speed = 0.12;
Ti_speed = 3.5;

% Position controller K
Kpos = 0.55;


% Speed PI controller:

% This is a combination of Kp + 1/Ti * s but simulink has the Ti inputted
% as I=Ti instead of I=1/Ti. So simulink and matlab models don't match each
% other.

% In both cases the complementary function of speed or position stayed a
% bit under 0 which means that the reference value can't be followed. In I
% = 1/Ti the speed had small negative value in where it should have been 0
% dB.

% I tried the command dcgain(sys_controlled_speed) to see the steady-state
% error. The steady-state value was 0.999999999999429 so I think it's not a
% big deal that the speed doesn't reach very exactly the point. But if the
% gains can be changed to better ones with little effort then why not?

PI=tf([K_p_speed*Ti_speed 1],[Ti_speed 0])
%PI=tf([K_p_speed Ti_speed],[1 0]) % here's the version with I=Ti (having
% I = 1/Ti would be the corrent presentation, though.

% Open loop speed transfer function + controller:
sysser_speed = T_N*PI*sys2_speed

% Open loop position transfer function + controller:
% It has closed-loop speed transfer function inside it.
% Remember to integrate the result to get speed -> position! 
% (Integrating: divide the result with s)
% (I'll put a drawing of this somewhere.)
s = tf('s'); 
sysser_pos = Kpos * feedback(T_N*PI*sys2_speed,1) * (1/s)


%% Sensitivity and complementary sensitivity
% Digital Control Design 2.2
% Loop transfer function: L=G*K (G is the system, K is the controller)
% K and G are different for speed and position because of the cascade connection.

Lpos = sysser_pos;
Lspeed = sysser_speed;

% Sensitivity:
%S=(1+L)^-1;

Spos   = (1+Lpos)^-1
Sspeed = (1+Lspeed)^-1

% Complementary sensitivity:
%T=G*K*S;

Tpos = Lpos*Spos
Tspeed = Lspeed*Sspeed

figure();
hold on
bode(Spos,'r')
bode(Tpos,'g')
%bode(Lpos)
hold off
title('Sensitivity (position)')
legend('Sensitivity', 'Complementary sensitivity')

figure();
hold on
bode(Sspeed,'r')
bode(Tspeed,'g')
%bode(Lspeed)
hold off
title('Sensitivity (speed)')
legend('Sensitivity', 'Complementary sensitivity')

figure
margin(sysser_speed)
title('Speed margin (open loop)')
% There's a spike around 3 kHz at the moment. The same spike is also in the
% complementary sensitivity figures but the peak is until 0 dB. Maybe
% something can be done with it, maybe not.

%% Speed control
% Closed loop:
sys_controlled_speed = feedback(sysser_speed, 1)
figure();
step(sys_controlled_speed)
title('Step response for speed controlled with PI controller')


%% Position control
% velocity to pos is integration after velocity and then pos control gain
% cascaded with rotational speed control
%  posC=Kpos*(1/s)
% refp      /------\        /-------\   /-------\     /--------\
% --+O-----|  Kpos |--+O---|  PI     | -|T2speed |---|speed2po-|----cartpos
%   -|     \-------/  -|    \-------/   \-------/  |  \--------/  |
%    |                 |___________________________|              |   
%    |____________________________________________________________|  


% sysser_pos was formed earlier and it contains:
% the position controller,
% the system with speed feedback and PI controller
% (1/s) term

% -> So there's no need to add anything here to it.

%Closed loop:
sys_controlled_pos =  feedback(sysser_pos,1)
figure();
margin(sysser_pos)
title('Position margin')

figure();
step(sys_controlled_pos)
title('Step response for position controlled with Kpos and PI-controller')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This is where I'll continue from tomorrow %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% (and maybe I might try to change the controller terms a bit)

%% Assignment part III
% Codesys file external file.

%% Assignment part V

%% SysID, tf model (sys2_speed / sys2_pos)
% Test (OE,TF,Impulse) system identification methods to state space model

% generate test data:
% https://se.mathworks.com/help/ident/ref/idinput.html
% 'rbs' — Random binary signal

% N data lenght (same than record length)
N = 835;

u = idinput(N,'rbs');

Ts=0.01
% create time axis
t=0:Ts:Ts*(N-1);

% Simulation (speed)

% relation between model and validation (835 samples)
model_samples= 300; % model, rest for validation
 
[y,t]=lsim(sys2_speed,u,t');

all_data_speed = iddata(y,u,Ts);
model_data_speed = all_data_speed(1:model_samples);
validation_data_speed = all_data_speed(model_samples+1:N);
get(all_data_speed);
model_data_speed.InputName = 'Torque';
model_data_speed.OutputName = 'Speed';
model_data_speed.TimeUnit = 'Seconds';
model_data_speed.InputUnit = 'Nm';
model_data_speed.OutputUnit = 'm/s';

validation_data_speed.InputName = 'Torque';
validation_data_speed.OutputName = 'Speed';
validation_data_speed.TimeUnit = 'Seconds';
validation_data_speed.InputUnit = 'Nm';
validation_data_speed.OutputUnit = 'm/s';


% Simulation (position)

[y,t] = lsim(sys2_pos,u,t');
all_data_pos = iddata(y,u,Ts);

model_data_pos = all_data_pos(1:model_samples);
validation_data_pos = all_data_pos(model_samples+1:N);
model_data_pos.InputName = 'Torque';
model_data_pos.OutputName = 'Position';
model_data_pos.TimeUnit = 'Seconds';
model_data_pos.InputUnit = 'Nm';
model_data_pos.OutputUnit = 'm';

validation_data_pos.InputName = 'Torque';
validation_data_pos.OutputName = 'Position';
validation_data_pos.TimeUnit = 'Seconds';
validation_data_pos.InputUnit = 'Nm';
validation_data_pos.OutputUnit = 'm';

figure
plot(model_data_speed)
figure
plot(model_data_pos)

%detrend
%hold on;
%getTrend(model_data_speed(1:model_samples))
%model_data_speed = detrend(model_data_speed)
%plot(model_data_speed(1:model_samples),'k*')
%getTrend(model_data_pos(1:model_samples))
%model_data_pos = detrend(model_data_pos)
%plot(model_data_pos(1:model_samples),'k*')


%% OE model

%OE method:
model_oe_speed = oe(model_data_speed, [5 3 1])
figure
compare(validation_data_speed, model_oe_speed)

model_oe_pos = oe(model_data_pos, [5 3 1])
figure
compare(validation_data_pos, model_oe_pos)

%% TF est

model_tf_speed = tfest(model_data_speed, 6,5,NaN)
model_tf_pos = tfest(model_data_pos, 6,5,NaN)

compare(validation_data_speed, model_tf_speed)

figure
compare(validation_data_pos, model_tf_pos)

figure;
bode(sys2_speed,'r',model_oe_speed,'k+',model_tf_speed,'y*')
bode(sys2_pos, 'r', model_oe_pos,'k+',model_tf_pos,'y*')
legend('Original','model_oe_pos','model_tf_speed')

%% impulse est

model_impulse_speed = impulseest(model_data_speed);
getpar(model_impulse_speed);
figure
compare(validation_data_speed,model_impulse_speed)

model_impulse_pos = impulseest(model_data_pos);
getpar(model_impulse_pos);
figure
compare(validation_data_pos,model_impulse_pos)
figure
compare(validation_data_speed,model_data_speed)



%%
% Comparison of different test data
% Load PRBS test data
load outputPRBS_1500.txt
data= outputPRBS_1500
speed1=outputPRBS_1500(:,4)
torque1=outputPRBS_1500(:,6)

%load Chirp test data 
%load outputSWEEP_10.txt
%data=outputSWEEP_10


%load outputSWEEP_20_1_20.txt
%data=outputSWEEP_20_1_20

%load outputSWEEP_20_1_100.txt
%data=outputSWEEP_20_1_100

%load outputSWEEP_20_1_1000.txt
%data=outputSWEEP_20_1_1000

% second best
%load outputSWEEP_20_1_2000.txt
%data=outputSWEEP_20_1_2000

torque1=data(:,6)
speed1=data(:,4)


Ts = 0.01

plot(speed1)
hold on;plot(torque1)
title('Real data from manipulator x-axis')
legend('Speed','Torque')

%data for model and validation:
all_data = iddata(speed1,torque1,Ts);
model_data = iddata(speed1(1:model_samples),torque1(1:model_samples),Ts);
validation_data = iddata(speed1(model_samples+1:N),torque1(model_samples+1:N),Ts);

get(model_data)
model_data.InputName = 'Torque';
model_data.OutputName = 'Speed';
model_data.TimeUnit = 'Seconds';
model_data.InputUnit = 'Nm';
model_data.OutputUnit = 'm/s';

validation_data.InputName = 'Torque';
validation_data.OutputName = 'Speed';
validation_data.TimeUnit = 'Seconds';
validation_data.InputUnit = 'Nm';
validation_data.OutputUnit = 'm/s';


%detrend
%hold on;
getTrend(model_data(1:model_samples))
model_data = detrend(model_data)
%plot(model_data(1:model_samples),'k*')

%% OE
% https://se.mathworks.com/help/ident/ref/oe.html


nb = 8;
nf = 7;
nk = 1;



model_oe = oe(model_data,[nb nf nk]);
compare(validation_data,model_oe)
%% Transfer function
% https://se.mathworks.com/help/ident/ref/tfest.html

np = 7;
nz = 6;
iodelay =NaN;

model_tf = tfest(model_data,np,nz,iodelay)
compare(validation_data, model_tf)
%resid(all_data,model_tf)

%% SS estimate
% https://se.mathworks.com/help/ident/ref/ssest.html
nx = 1:20;
model_ss = ssest(model_data,nx)
compare(validation_data,model_ss)

%% impulse estimation
%https://se.mathworks.com/help/ident/ref/impulseest.html
model_impulse = impulseest(model_data)
compare(validation_data,model_impulse)
%resid(all_data,model_impulse)

%% create ARX model and test "whiteness of residuals"

% Min squared residual estimation
m_arx = arx(model_data,[10 10 7]);   %[na nb nk])
present(m_arx)  % look at ARX model
figure;
compare(validation_data, m_arx)
%resid(model_data,m_arx)


%% nlarx
% https://se.mathworks.com/help/ident/ref/nlarx.html

names = [model_data.OutputName, model_data.InputName];

output_lag = [1 2 3 4];
input_lag = [1 2 3 4 5];
lags = {output_lag,input_lag};

L = linearRegressor(names,lags);
NL1 = nlarx(model_data, L, idLinear );
NL1.OutputFcn

compare(validation_data, NL1)

%m1_arx_nl = nlarx(model_data,m_arx,)
%present(m1_arx_nl)
%m0_arx_nl  = init(m1_arx_nl)
%m2_arx_nl = nlarx(model_data,m0_arx_nl)
%present(m2_arx_nl)
%compare(validation_data,m2_arx_nl)
%resid(validation_data,m2_arx_nl)


%% Comparison
% Comparing OE and TF method estiation to space state model using same RBS
% input signal. The result of OE and TF is quite far from perfect. OE and
% TF can follow somehow but the model is missing some dynamics that causes 
% oscillation. That may caused by the another axis.

% change model
%[y,t]=lsim(model_oe,u,t');
[y,t]=lsim(model_tf,u,t');
org_model_data = iddata(y,u,Ts)

[y,t]=lsim(sys2_speed,u,t');
own_model_data = iddata(y,u,Ts)

compare(org_model_data,own_model_data)