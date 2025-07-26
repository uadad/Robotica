
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulaci�n del movimiento de un robot m�vil
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
clc

j=1;

global l
global radio_rueda
global camino
global pose
global punto
%cargamos el camino

camino=load('camino.dat');

l=3.5; %distancia entre rudas delanteras y traseras, tambien definido en modelo
radio_rueda=1;

%Condiciones iniciales 
pose0=[0; 0; 0];

t0=0;

%final de la simulaci�n
tf=30;

%paso de integracion
h=0.1;
%vector tiempo
t=0:h:tf;
%indice de la matriz
k=0;

%inicializaci�n valores iniciales
pose(:,k+1)=pose0;

t(k+1)=t0;

while (t0+h*k) < tf
    %actualizaci�n
    k=k+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %valores de los par�metros de control
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    velocidad=2;
    R=10;
    rho=1/R;
    velocidad_derecha=velocidad*(1+l*rho)/radio_rueda;
    velocidad_izquierda=velocidad*(1-l*rho)/radio_rueda;
    phi=atan(rho*l);
    %volante=-0.1416;
    volante=phi;
    conduccion=[velocidad_derecha velocidad_izquierda];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %para representar el punto onjetivo sobre la trayectoria
    punto=[30 30];
    %metodo de integraci�n ruge-kuta
    pose(:,k+1)=kuta_diferencial(t(k),pose(:,k),h,conduccion);

end




