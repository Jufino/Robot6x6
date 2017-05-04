close all;
K = 1.165;
T = 0.090;
periodaCurrent = 0.0015;
PCurrent = 2;
ICurrent = 0.080;
DCurrent = 0;
kpCurrent = PCurrent;
kiCurrent = PCurrent*(periodaCurrent/ICurrent);
kdCurrent = PCurrent*(DCurrent/periodaCurrent);

q0= (kpCurrent+kdCurrent);
q1= (-kpCurrent-2*kdCurrent+kiCurrent);
q2= kdCurrent;
p1= ((kiCurrent-kdCurrent)/(kpCurrent+kiCurrent+kdCurrent));
p2= ((kdCurrent)/(kpCurrent+kiCurrent+kdCurrent));

x = 0:1:600;
x = x.*0.0015;

y50 = [0;csvread('regcurrent50')];
plot(x,y50)

hold on
stepValue = 50;
sim('psd_regulator_prudu')
tSim50 = tSim;
ySim50 = ySim;
plot(tSim50,ySim50)

hold on
y80 = [0;csvread('regcurrent80')];
plot(x,y80)

hold on
stepValue = 80;
sim('psd_regulator_prudu')
tSim80 = tSim;
ySim80 = ySim;
plot(tSim80,ySim80)

sumaStvorcov = sum((ySim80-y80).^2+(ySim50-y50).^2)



grid
legend('žiadana hodnota = 50 mA(namerané)','žiadana hodnota = 50 mA(simulink)','žiadana hodnota = 80 mA(namerané)','žiadana hodnota = 80 mA(simulink)')
title('Priebeh výstupnej velièiny PSD regulátora prúdu');
xlabel('èas [s]');
ylabel('prúd [mA]');
