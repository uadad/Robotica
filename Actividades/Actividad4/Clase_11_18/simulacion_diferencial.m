
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulación del movimiento de un robot móvil
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

%final de la simulación
tf=30;

%paso de integracion
h=0.1;
%vector tiempo
t=0:h:tf;
%indice de la matriz
k=0;

%inicialización valores iniciales
pose(:,k+1)=pose0;

t(k+1)=t0;

while (t0+h*k) < tf
    %actualización
    k=k+1;  
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %valores de los parámetros de control
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    punto=[30 5];
    delta=(pose(1,k)-punto(1))*sin(pose(3,k))-((pose(2,k)*punto(2))*cos(pose(3,k)));
    L_h=sqrt(((pose(1,k)-punto(1))^2)+((pose(2,k)-punto(2))^2));
    distancia=sqrt(((pose(1,k)-punto(1))^2)+((pose(2,k)-punto(2))^2));
    rho=2*delta/(L_h^2);
   
    %kpv=1;
    velocidad = 5;
    %velocidad=kpv*distancia;

    velocidad_derecha=velocidad*(1+l*rho)/radio_rueda;
    velocidad_izquierda=velocidad*(1-l*rho)/radio_rueda;
    conduccion=[velocidad_derecha velocidad_izquierda];
    %metodo de integración ruge-kuta
    pose(:,k+1)=kuta_diferencial(t(k),pose(:,k),h,conduccion);

end




