%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Ejemplo de uso del algorithmo A* en un mapa definido por un bmp 
% 21-12-2015
%   fgb
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%clear all
clc
%el fichero BMP se carga en el fichero principal

 MAPA = imread('cuadro4.bmp');
 global MAPA

%Transformación para colocar correctamente el origen del Sistema de
%Referencia
 MAPA(1:end,:,:)=MAPA(end:-1:1,:,:);

pose_ini=[212 52];

pose_dest=[513 490];

%Tamaño de las celdas del grid
delta=25;

%delta=35;

%Llamada del algoritmo

Optimal_path=A_estrella_modificado_2(MAPA,delta,pose_ini,pose_dest);

%Dibujo de la ruta
plot(Optimal_path(:,1), Optimal_path(:,2))
% ds=1;
% curva=funcion_spline_cubica_varios_puntos(Optimal_path(:,1)',Optimal_path(:,2)',ds);
% plot(curva(1,:),curva(2,:),'r')
% camino=curva';

