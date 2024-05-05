clc; clear; close all;

% Fungsi transfer Plant
Ts = 0.01;
J = 0.01;
b = 0.1;
Ke = 0.01;
Kt = 0.01;
R = 1;
L = 0.5;
syms s;
K = Ke;
num = K;
den = sym2poly((J*s+b)*(L*s+R)+K^2);
sys = tf(num,den);
Plant = c2d(sys,Ts,'zoh');
figure;
step(Plant);
title('Step Response');

open_system('SimFuzzyPID');
open_system('SimFuzzyPID/Fuzzy PID');

% Mendesain kendali PID konvensional
open_system('SimFuzzyPID/PID');

C0 = pid(1,1,1,'Ts',Ts,'IF','B','DF','B'); % PID structure
C = pidtune(Plant,C0); % design PID
[Kp, Ki, Kd] = piddata(C); % Parameter PID

% Asumsikan sinyal referensi bernilai 1, sehingga max. error |e|=1 
% Rentang input |E| adalah [-10 10], sehingga atur |GE| = 10.

GE = 100;
GCE = GE*(Kp-sqrt(Kp^2-4*Ki*Kd))/2/Ki; % Kp = GCU * GCE + GU * GE
GCU = Ki/GE; % Ki = GCU * GE
GU = Kd/GCE; % Kd = GU * GCE

% Fuzzy inference system Sugeno:
FIS = sugfis;  % Yeni Sugeno tipi fuzzy inference sistemi oluştur

% Fungsi keanggotaan input error |E|:
FIS = addInput(FIS,[-100 100],'Name','E');
FIS = addMF(FIS,'E','gaussmf',[70 -100],'Name','Negative');
FIS = addMF(FIS,'E','gaussmf',[70 100],'Name','Positive');

% Fungsi keanggotaan input perubahan error |CE|:
FIS = addInput(FIS,[-100 100],'Name','CE');
FIS = addMF(FIS,'CE','gaussmf',[70 -100],'Name','Negative');
FIS = addMF(FIS,'CE','gaussmf',[70 100],'Name','Positive');

% Fungsi keanggotaan output |u|:
FIS = addOutput(FIS,[-200 200],'Name','u');
FIS = addMF(FIS,'u','constant',-200,'Name','Min');
FIS = addMF(FIS,'u','constant',0,'Name','Zero');
FIS = addMF(FIS,'u','constant',200,'Name','Max');

% Aturan Fuzzy
ruleList = [1 1 1 1 1;  % If |E| is Negative and |CE| is Negative then |u| is -200 (MIN)
            1 2 2 1 1;  % If |E| is Negative and |CE| is Positive then |u| is 0 (ZERO)
            2 1 2 1 1;  % If |E| is Positive and |CE| is Negative then |u| is 0 (ZERO)
            2 2 3 1 1]; % If |E| is Positive and |CE| is Positive then |u| is 200 (MAX)
FIS = addRule(FIS, ruleList);

sim('SimFuzzyPID');
load('StepPID');
load('StepFP');
figure;
if length(StepPID) > 400 && length(StepFP) > 500
    plot(StepPID(1, 1:401), StepPID(2, 101:501));
    hold on;
    plot(StepFP(1, 1:401), StepFP(2, 101:501));
else
    plot(StepPID(1, :), StepPID(2, :));  % Tüm verileri çizdir
    hold on;
    plot(StepFP(1, :), StepFP(2, :));  % Tüm verileri çizdir
end
hold off;
title('System Response');
legend('PID', 'Fuzzy-PID');

% Load simulation data
load('StepPID');
load('StepFP');

% Ensure that the data vectors are not exceeding their bounds and are of equal length
lenPID = size(StepPID, 2);
lenFP = size(StepFP, 2);
minLength = min(lenPID, lenFP);  % Find the minimum length to safely index both arrays

% Adjust indices to avoid out of bounds error and ensure vector length equality
startIndexPID = max(101, minLength - 400);  % Ensure we have enough data points to display
endIndexPID = min(startIndexPID + 400, minLength);

startIndexFP = max(101, minLength - 400);
endIndexFP = min(startIndexFP + 400, minLength);

% Plotting
figure;
plot(StepPID(1, startIndexPID:endIndexPID), StepPID(2, startIndexPID:endIndexPID));
hold on;
plot(StepFP(1, startIndexFP:endIndexFP), StepFP(2, startIndexFP:endIndexFP));
hold off;

title('Response System');
legend('PID', 'Fuzzy-PID');
xlabel('Time');
ylabel('Response');

% Display for debugging
disp(['StepPID vector length: ', num2str(lenPID)]);
disp(['StepFP vector length: ', num2str(lenFP)]);
disp(['Using indices: ', num2str(startIndexPID), ' to ', num2str(endIndexPID)]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%