
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

%camino=load('camino.dat');

x=20:0.1:60;
y=zeros(size(x));
camino=[x' y'];

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

V=10;
look_ahead=20;

while (t0+h*k) < tf
    %actualización
    k=k+1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %valores de los parámetros de control
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   
    punto=pure_presuit(camino,pose(:,k),look_ahead);
    punto_final=camino(length(camino),:);
    %[V p]=funcion_controlador_geometrico(pose(:,k),punto);
    [V p]=funcion_controlador_geometrico2(pose(:,k),punto,punto_final);
    [velocidad_derecha velocidad_izquierda]=funcion_modelo_cinematico_inverso(V,p);
    conduccion=[velocidad_derecha velocidad_izquierda];
 
    %metodo de integración ruge-kuta

    pose(:,k+1)=kuta_diferencial(t(k),pose(:,k),h,conduccion);
end



