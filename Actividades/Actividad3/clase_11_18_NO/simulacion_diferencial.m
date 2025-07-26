
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulación del movimiento de un robot móvil
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
clc

joy = vrjoystick(1);

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
tf=50;

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
    velocidad=2;
    R=10;
 
    rho=1/R;

velocidad_derecha=velocidad*(1+l*rho)/radio_rueda;

 velocidad_izquierda=velocidad*(1-l*rho)/radio_rueda;
 
 phi=atan(rho*l);
 
 volante=-0.1416;
 
 volante=phi;
 
 velocidad=2;


 
 conduccion=[velocidad_derecha velocidad_izquierda];
 
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
 %para representar el punto onjetivo sobre la trayectoria
 
 punto=[30 30];

    
 velocidad(k)=-5*axis(joy,5); %velocidad lineal
 w(k)=-axis(joy,1);   %velocidad angular
%metodo de integración ruge-kuta

pose(:,k+1)=kuta_diferencial(t(k),pose(:,k),h,conduccion);

end



%plot(t(1:end-1),velocidad);

%hold on plot(t(1:end-1),w);

% t(1:end-1),w./velocidad); % curvatura



% alfa1=(v/r)*(1+l*p)=1/r *(v+l*w)