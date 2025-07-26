matriz_pinza=eye(4,4) % matriz unidad
q=[0 -1.5700 -1.5700 -1.5700 1.5700 0];
matriz=funcion_pinta_UR3_new(q, matriz_pinza)

%% Ejecicio A)
%configuración de la pieza
posicion=[20 -10 0];
alfa=0; beta=0; gamma=0;
  
matriz_pieza=Desplazamiento(posicion(1), posicion(2), posicion(3))*Rotacionz(alfa)*Rotaciony(beta)*Rotacionx(gamma);

%pinta_pieza_delgada(matriz_pieza)
pinta_bloque(matriz_pieza,'b')

%% Ejecicio B)
clear all
clc 
%pinta_bloque(Desplazamiento(8,8,8),'b');
alfa=0; beta=0; gamma=0; % Angulos de Euler
posicion=[20 -10 0];

matriz_pieza=Desplazamiento(posicion(1), posicion(2), posicion(3))*Rotacionz(pi/4);
matriz_pinza=eye(4,4); % matriz unidad

pinta_bloque(matriz_pieza,'b')

matriz_agarre = Desplazamiento(0,0,4)*Rotacionz(pi/2)*Rotacionx(pi); % *Rotaciony(pi/2)*Rotacionx(-pi/2)
% matriz_agarre = Desplazamiento(0,0,4)*Rotacionz(pi/4)*Rotacionz(-pi/2)*Desplazamiento(0,5,0)

matriz_pinza = matriz_pieza*matriz_agarre;
q=[0 -1.5700 -1.5700 -1.5700 1.5700 0];
matriz=funcion_pinta_UR3_new(q, matriz_pinza)
%% apartado 2 de B
clear all
clc 
%pinta_bloque(Desplazamiento(8,8,8),'b');
alfa=0; beta=0; gamma=0; % Angulos de Euler
posicion=[20 -10 0];

matriz_pieza=Desplazamiento(posicion(1), posicion(2), posicion(3))*Rotacionz(pi/4);
matriz_pinza=eye(4,4); % matriz unidad

pinta_bloque(matriz_pieza,'b')

matriz_agarre = Desplazamiento(0,0,4)*Rotacionz(pi)*Rotaciony(pi); % *Rotaciony(pi/2)*Rotacionx(-pi/2)


matriz_pinza = matriz_pieza*matriz_agarre;
q=[0 -1.5700 -1.5700 -1.5700 1.5700 0];
matriz=funcion_pinta_UR3_new(q, matriz_pinza);


%% Apartado D)

clear all
clc 
%pinta_bloque(Desplazamiento(8,8,8),'b');
alfa=pi/4; beta=0; gamma=pi/6; % Angulos de Euler
posicion=[20 10 5];

matriz_pieza=Desplazamiento(posicion(1), posicion(2), posicion(3))*Rotacionz(pi/4);
matriz_pinza=eye(4,4); % matriz unidad

pinta_bloque(matriz_pieza,'b')

matriz_agarre = Desplazamiento(0,0,4)*Rotacionz(pi)*Rotaciony(pi); % *Rotaciony(pi/2)*Rotacionx(-pi/2)


matriz_pinza = matriz_pieza*matriz_agarre;
q=[0 -1.5700 -1.5700 -1.5700 1.5700 0];
matriz=funcion_pinta_UR3_new(q, matriz_pinza);





%% Apartado E)

clear all
clc 
%pinta_bloque(Desplazamiento(8,8,8),'b');
alfa=0; beta=0; gamma=0; % Angulos de Euler
posicion=[20 -10 0];

matriz_pieza=Desplazamiento(posicion(1), posicion(2), posicion(3))*Rotacionz(pi/4);
matriz_pinza=eye(4,4); % matriz unidad

pinta_bloque(matriz_pieza,'b')

matriz_agarre = Desplazamiento(0,-0.8,4)*Rotaciony(pi/2)*Rotacionx(-pi/2)


matriz_pinza = matriz_pieza*matriz_agarre;
q=[0 -1.5700 -1.5700 -1.5700 1.5700 0];
matriz=funcion_pinta_UR3_new(q, matriz_pinza);



clear all
clc 
%pinta_bloque(Desplazamiento(8,8,8),'b');
alfa=pi/4; beta=0; gamma=0; % Angulos de Euler
posicion=[20 -10 0];

matriz_pieza=Desplazamiento(posicion(1), posicion(2), posicion(3))*Rotacionz(pi/4);
matriz_pinza=eye(4,4); % matriz unidad

pinta_bloque(matriz_pieza,'b')

matriz_agarre = Desplazamiento(0,-0.8,4)*Rotaciony(pi/2)*Rotacionx(-pi/2)


matriz_pinza = matriz_pieza*matriz_agarre;
q=[0 -1.5700 -1.5700 -1.5700 1.5700 0];
matriz=funcion_pinta_UR3_new(q, matriz_pinza);



