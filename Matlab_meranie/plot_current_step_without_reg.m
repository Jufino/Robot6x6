x = 0:1:600;
x = x.*0.001;

u25 = [0;ones(600,1)*25];
y25 = [0;csvread('skok25')];

u50 = [0;ones(600,1)*50];
y50 = [0;csvread('skok50')];

u75 = [0;ones(600,1)*75];
y75 = [0;csvread('skok75')];

u100 = [0;ones(600,1)*100];
y100 = [0;csvread('skok100')];

u125 = [0;ones(600,1)*125];
y125 = [0;csvread('skok125')];

u150 = [0;ones(600,1)*150];
y150 = [0;csvread('skok150')];

u175 = [0;ones(600,1)*175];
y175 = [0;csvread('skok175')];

u200 = [0;ones(600,1)*200];
y200 = [0;csvread('skok200')];

u225 = [0;ones(600,1)*225];
y225 = [0;csvread('skok225')];

u255 = [0;ones(600,1)*255];
y255 = [0;csvread('skok255')];

plot(x,y25)
hold on
plot(x,y50)
hold on
plot(x,y75)
hold on
plot(x,y100)
hold on
plot(x,y125)
hold on
plot(x,y150)
hold on
plot(x,y175)
hold on
plot(x,y200)
hold on
plot(x,y225)
hold on
plot(x,y255)
hold on

grid
legend('skok z 0 na 10% PWM','skok z 0 na 20% PWM','skok z 0 na 30% PWM','skok z 0 na 40% PWM','skok z 0 na 50% PWM','skok z 0 na 60% PWM','skok z 0 na 70% PWM','skok z 0 na 80% PWM','skok z 0 na 90% PWM','skok z 0 na 100% PWM')
title('Priebeh prúdu v závislosti na èase a ve¾kosti skoku');
xlabel('èas [s]');
ylabel('prúd [mA]');