%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Ejemplo de uso del algorithmo A* en un mapa definido por un bmp 
% 21-12-2015
%   fgb
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
clc
%Carga el fichero  BMP
MAPA = imread('.\cuadro4.bmp');
%Transformación para colocar correctamente el origen del Sistema de
%Referencia
MAPA(1:end,:,:)=MAPA(end:-1:1,:,:);
%Tamaño de las celdas del grid
delta=20;  % el tamaño de cuadricula, si es muy pqueña puede ocurrir que la navegacion no sea segura, y el tiempo se dispara
%Llamada del algoritmo
Optimal_path=A_estrella(MAPA, delta); % optimal_path es un array que contiene todos los puntos del camino

%Dibujo de la ruta
plot(Optimal_path(:,1), Optimal_path(:,2),'*r')

