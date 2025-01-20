%Require System Identification Toolbox, Control System toolbox and PID Tuner app

clc
% Specify the file name
filenameR2 = 'motorR_PWM_speed2.csv'; % Replace with csv file name % file must content column header title

% Import data
dataR2 = readtable(filenameR2);

% Extract PWM and MotorSpeed columns
PWMR2 = dataR2.PWMR2;  % Input (u) %csv file input column header
MotorSpeedR2 = dataR2.MotorSpeedR2;  % Output (y) %csv file output column header

% Define sampling time (Ts) - Adjust based on your data's time resolution
Ts = 0.01; % Example: 0.01 seconds

% Create the iddata object
data_R2_rampup = iddata(MotorSpeedR2, PWMR2, Ts);

%Create transfer function model 
np = [2]; % 2 poles(typical DC motor)
nz = [0];
sys_R2_rampup = tfest(data_R2_rampup, np, nz); % Motor insert iddata object

%open PID tuner
pidTuner(sys_R2_rampup, 'PID'); 