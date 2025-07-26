
clc, close all, clear all

% desde el punto 1 al 2
pose0=[15; 12; 0];
posef=[216; 250; pi/2];


direccion=1;

Actividad_7_v0_mapa

% desde el punto 2 al 3 marcha atras
pose0=pose(:,end);
posef=[237; 225; pi];

pose=[];
direccion=-1;

Actividad_7_v0_mapa

% desde el punto 3 al 4 hacia adelante
pose0=pose(:,end);
posef=[237; 77; 5*(pi/4)];

pose=[];
direccion=1;

Actividad_7_v0_mapa

% desde el punto 4 al 5 hacia adelante
pose0=pose(:,end);
posef=[56; 25; -pi/4];

pose=[];
direccion=1;

Actividad_7_v0_mapa

% ultimo paso aparcar
pose0=pose(:,end);
posef=[15; 12; 0];

pose=[];
direccion=-1;

Actividad_7_v0_mapa