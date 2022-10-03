%% Controllo Velocità Motore
%Luca Erviati Università degli Studi di Napoli Federico II

clear all, close all, clc

%%Parametri per Modello Motore
J = 0.02;
b = 0.2;
K = 0.01;
R = 0.4;
L = 0.7;
s = tf('s');
F_Motor = K/((J*s+b)*(L*s+R)+K^2);

% Inizializzazione sistema e calcolo risposta indiciale
G = F_Motor;

dt = 1e-3;
[y,t] = step(G,0:dt:10);

plot(t,y,'linewidth',2)
hold on, grid on
plot(t,t*0+y(end),'k')
title('Step response')

% Ricerca flesso
iflesso = find(diff(diff(y))<0,1);
tflesso = t(iflesso);
plot(tflesso,y(iflesso),'or')

% Calcolo tangente
m = (y(iflesso+1)-y(iflesso))/dt;
tau = tflesso - y(iflesso)/m;
plot(tau,0,'or')

% Calcolo costante di tempo
dT = tflesso + (y(end)-y(iflesso))/m;
plot(dT,dcgain(G),'or');
plot([tau, dT], [0 dcgain(G)], '--k');
T = dT - tau;

% Calcolo guadagno statico
mu = y(end)/1;

%% Calcolo controllore PID ZN

Kp = 1.2*T/(mu*tau);
Ti = 2*tau;
Td = 0.5*tau;


Ki = Kp/Ti;
Kd = Kp*Td;

N = 10;

s = tf('s');
PID = Kp + Ki/s + Kd*s/(1+s*Td/N);

clSys1 = feedback(G*PID,1);
figure(2)
hold on
Szn = step(clSys1);
[y,t] = step(clSys1);
plot(t, squeeze(y),'DisplayName','ZN','LineWidth',1.5)
grid on;



%% Calcolo controllore PID CC

Kp = (1.35/(tau/T) + 0.27)/mu;
Ti = (2.5*(tau/T)*(1+tau/T/5)/(1+0.6*(tau/T))) * T;
Td = (0.37*(tau/T)/(1+0.2*(tau/T)))*T;

Ki = Kp/Ti;
Kd = Kp*Td;

N = 10;

s = tf('s');
PID = Kp + Ki/s + Kd*s/(1+s*Td/N);

clSys2 = feedback(G*PID,1);
figure(2)
hold on

Scc = step(clSys2);
[y,t] = step(clSys2);
plot(t, squeeze(y),'DisplayName','CC','LineWidth',1.5)
grid on;
legend;



%% Plot PID GA

data = OUT.Data;
time = OUT.Time;
plot(time-1, data,'g','DisplayName','GA','LineWidth',1.5)
stepplot(clSys1,'r')
grid on;

%% Plot StepInfo

disp('RISPOSTA AL GRADINO: Ziegler-Nichols')
stepinfo(clSys1)
disp('Errore a regime percentuale ZN')
sseZN = abs((Szn(end)-1))./1*100;
disp(sseZN)
disp('RISPOSTA AL GRADINO: Cohen-Coon')
stepinfo(clSys2)
disp('Errore a regime percentuale CC')
sseCC = abs((Scc(end)-1))./1*100;
disp(sseCC)