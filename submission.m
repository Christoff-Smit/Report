%% Introduction
%Author: Christoff Smit
%Date: 2020-05-17
clc;

%% Declaration of variables

syms s

% Constants:
syms Km % motor constant
syms Kew KTw % motor speed constants
syms KTi % generator armature current constant
syms Kei % generator field current constant

% DC motor:
syms Vam Iam Ram Lam Em Wm Tm Pm % armature
syms J % moment of inertia
syms b % damping coefficient
syms G1 G2

% Synchronous generator:
syms Eg Rag RL Lag % armature
syms Vfg Ifg Rfg Lfg % field
syms G3 G4


%% DC motor Equations:
G1 = Km/(Ram + s*Lam);
G2 = 1/(J*s + b);

Em = Km*Wm;
Tm = Km*Iam;
Pm = Tm*Wm;


%% AC generator Equations:
G3 = 1/(Rfg + s*Lfg);
G4 = 1/(Rag + RL + s*Lag);



%% Plant Transfer Function (Mason):
%Forward Path:
P1 = G3*Kei*G4*RL;

%Loops:
L1 = -Km*G1*G2;
L2 = G2*-KTw*(-1);
L3 = KTi*(-1)*G2*Kew*G4;

delta = 1 - (L1+L2+L3) + (0); %all loops touch each other

delta1 = 1 - (L1+L2+0); %only loop L3 touches path P1

P_mason = (P1*delta1)/delta;
[num,den] = numden(collect(P_mason,s));

num_coeffs = coeffs(num,s);
den_coeffs = coeffs(den,s);
ascii_tab_char = 9;
disp(['order of the numerator: ',num2str(length(num_coeffs)-1),ascii_tab_char,'(',num2str(length(num_coeffs)),' coefficients)'])
disp(['order of the denominator: ',num2str(length(den_coeffs)-1),ascii_tab_char,'(',num2str(length(den_coeffs)),' coefficients)'])


%% Assigning values to parameters:


%Constants:
Km = 1.375198;

KTw = 0.05;

Kew = 0.134;

Kei = 724.65;

KTi = 0.0796;

%DC Motor parameters
Ram = 18.24; %ohm

Lam = 0.5;  %henry

J = 0.521;
J_tau = 1.1255;

b = 0.02;

Tm = 1.2;

%AC Generator parameters
Rfg = 320;

Lfg = 0.5;

Rag = 8.533;

% Lag = 0.1;
Lag = 1.8;

infinity = 5e2;

% RL = 64; %full-load
RL = 32; %half-load
% RL = infinity; %no-load

%no load:
if(RL==infinity)
    KTi = 1.9456;
    KTw = -0.016094;
    load_state = 'no load';
end
%loaded with RL=32:
if(RL==32)
    KTi = 1.35;
    KTw = -0.011167;
    load_state = 'half load';
end
%loaded with RL=64:
if(RL==64)
    KTi = 2.4088;
    KTw = -0.019926;
    load_state = 'full load';
end

disp(['RL = ',num2str(RL),' (',load_state,')'])


%% Continuous Plant TF
ns2 = double(subs(num_coeffs(3))); % numerator s^2 coefficient
ns1 = double(subs(num_coeffs(2))); % numerator s^1 coefficient
ns0 = double(subs(num_coeffs(1))); % numerator s^0 coefficient

ds4 = double(subs(den_coeffs(5))); % denominator s^4 coefficient
ds3 = double(subs(den_coeffs(4))); % denominator s^3 coefficient
ds2 = double(subs(den_coeffs(3))); % denominator s^2 coefficient
ds1 = double(subs(den_coeffs(2))); % denominator s^1 coefficient
ds0 = double(subs(den_coeffs(1))); % denominator s^0 coefficient

P_cont = tf([ns2 ns1 ns0],[ds4 ds3 ds2 ds1 ds0]);
disp('Continuous Plant TF:');
minreal(P_cont)

timeInSecs = 2;
t=0:0.01:timeInSecs;
figure('name',['Continuous Plant TF step and impulse response (', num2str(timeInSecs),' secs)']);
subplot(2,1,1);
step(P_cont,t);
subplot(2,1,2);
impulse(P_cont,t);

% figure('name','Continuous Plant TF Bode plot');
% bode(P_cont)
% grid on

% figure('name','Continuous Plant TF Root Locus plot');
% rlocus(P_cont)
% grid on
% [poles,zeros] = pzmap(P_cont)

% return


%% State-space model

state_space_model = ss(P_cont)
size(state_space_model)
% state_space_model.InputDelay = 2.7 %seconds

% return


%% Discrete Plant TF

P_cont_bandwidth = 1/(2*pi) * bandwidth(P_cont); %degrees/s
sampling_frequency = P_cont_bandwidth*100;
% sampling_frequency = 500; %Hz
sampling_rate = 1/sampling_frequency; %seconds per period

P_disc = c2d(P_cont,sampling_rate,'zoh');
% disp('Discrete Plant TF:');
% minreal(P_disc)
% return

stopTime = 0.15;
t=0:sampling_rate:stopTime;

% step(P_cont,P_disc,t);
% legend('Continuous', 'Discrete')
% title('Plant (half-load): continuous vs discrete step response');

% figure
% subplot(2,1,1);
% step(P_disc,t);
% subplot(2,1,2);
% impulse(P_disc,t);

% bode(P_disc)
% grid on

% pzmap(P_disc)
% grid on
% [poles,zeros] = pzmap(P_disc)

% bode(P_cont,'k',P_disc,'r--')
% legend('Continuous', 'Discrete')
% grid on

% return


%% PID Controller

% %TEST:
% % s = tf('s');
% % P_cont = 1/(s^2 + 10*s + 20);
% P_cont = tf([1],[1 10 20]);
% % Kp = 350;
% % Ki = 300;
% % Kd = 50;
% % Kp = 1; %proportional gain
% % Ki = 1; %integral component
% % Kd = 1; %derivative component
% Kp = 39.2721; %proportional gain
% Ki = 115.9397; %integral component
% Kd = 3.3256; %derivative component
% D_s = pid(Kp,Ki,Kd);
% tf(D_s);
% system_forward_path = D_s*P_cont;
% system_feedback_loop = 1;
% T = feedback(system_forward_path,system_feedback_loop,-1)
% t = 0:0.01:2;
% step(T,t)
% % title('myTitle');
% return
% %END OF TEST

%Initial guesses:
% Kp = 1; %proportional gain
% Ki = 1; %integral component
% Kd = 1; %derivative component

% Kp = 200; %proportional gain
% Ki = 110000; %integral component
% Kd = 0.1; %derivative component

%Tuned path gains:
Kp = 0.86867; %proportional gain
Ki = 60.5979; %integral component
Kd = 0.0017569; %derivative component

D_s = pid(Kp,Ki,Kd);
% disp('Continuous Controller TF:');
% tf(D_s)
% return

system_feedback_loop = 1;

system_forward_path = D_s*P_cont;
T_s = feedback(system_forward_path,system_feedback_loop,-1);
% display('SYSTEM TF (continuous):');
% minreal(T_s)

% step(T_s)
% title('System: continuous step response')
%       OR
% stopTime = 0.2;
% t = 0:sampling_rate:stopTime;%time vector
% step(T_s,t)
% title('continuous step response')


D_z = pid(Kp,Ki,Kd,'Ts',sampling_rate)
disp('Discrete Controller TF:');
tf(D_z)
% return

system_forward_path = D_z*P_disc;
T_z = feedback(system_forward_path,system_feedback_loop,-1);
% display('SYSTEM TF (discrete):');
% minreal(T_z)
figure
step(T_s,T_z,t)
title('System: continuous vs discrete step response');
legend('continuous','discrete');










