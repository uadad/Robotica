
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
tf=32;

%paso de integracion
h=0.1;
%vector tiempo
t=0:h:tf;
%indice de la matriz
k=0;

%inicialización valores iniciales
pose(:,k+1)=pose0;

t(k+1)=t0;

puntos=[camino(500,:); camino(800,:); camino(1200,:)];

i=1;

while (t0+h*k) < tf
    %actualización
    k=k+1;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %valores de los parámetros de control
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %punto=[80 80]
    %punto=[camino(k,1) camino(k,2)];
    if i>3
        i=3;
    end
    punto=puntos(i,:);

    [V p]=funcion_controlador_geometrico(pose(:,k),punto);
    [velocidad_derecha velocidad_izquierda]=funcion_modelo_cinematico_inverso(V,p);
    conduccion=[velocidad_derecha velocidad_izquierda];
    %metodo de integración ruge-kuta
    pose(:,k+1)=kuta_diferencial(t(k),pose(:,k),h,conduccion);
    if V<0.01 % para no coger la distancia, ya que la velocidad es proporcional a la distancia
        i=i+1;
    end
end



