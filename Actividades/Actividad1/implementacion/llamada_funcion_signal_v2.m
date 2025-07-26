
clc
clear all
clc
%datos para dibuja la señal de referencia
Delay=2;
Periodo=6;
Amplitud=pi/2;
tiempo_simulado=0:0.01:4*pi;
y=signal_vf_v2(tiempo_simulado,Periodo,Delay,Amplitud);
plot(tiempo_simulado,y)
