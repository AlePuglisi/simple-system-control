 clc
clear 
close all 

%% SIMPLE SYSTEM TUNING 

% system data
m = 1.0;  % mass [kg]
l = 1.0;  % cylindrical load lenght [m]
R = 0.05; % cylindircal load radius[m]

% Inertia properties
J = 1/4*m*R^2+1/3*m*l^2; % cylindircal load inertia (around rotating axis)
N = 250;                 % reduction ratio of gearbox (not used) 
mu = 1/J;                % system TF gain 

% P+PI tuning (user defined cut-off frequency) 
% inner loop velocity cut-off frequency
wcv = 50;      % [Hz]
wcv = wcv*2*pi; % [rad/s]
% outer loop position cut-off frequency
wcp = 5;       % [Hz]
wcp = wcp*2*pi; % [rad/s]

% P+PI gains
Kpv = wcv/mu;       % proportional velocity gain
Tiv = 1/(0.1*wcv);  % integral velocity time
Kpp = wcp;          % proportional position gain
                          

% Additional gains
Tc  = 0.5;              % back-calculation anti wind up time constant
wff = 50;               % velocity feed forward refrence LP filter cut-off[Hz]
Tff = 1/(wff*2*pi);     % wff converted as TF time constant
wlpf = 5;               % position refernce pre-filter cut-off [Hz]
Tlpf = 1/(wlpf*2*pi);   % wlpf converted as TF time constant

L = 30;                 % gain used for a de-saturation anti-wind up implementation

%% BODE ANALYSIS 

% H = tf([Tiv*mu*Kpv, mu*Kpv], [Tiv, 0, 0]);
% figure(1)
% margin(H);
% F_v = H/(1+H);
% figure(2)
% bode(F_v)
% K = Kpp*F_v*tf([1],[1,0]); 
% figure(3)
% margin(K);

