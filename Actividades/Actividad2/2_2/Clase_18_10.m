
matriz_pinza=eye(4,4) % matriz unidad
q=[0 -1.5700 -1.5700 -1.5700 1.5700 0];
matriz=funcion_pinta_UR3_new(q, matriz_pinza)

%pieza1
posicion=[30,0,0];
alfa=0; beta=0; gamma=0;

%pieza2
posicion2=[20,20,0];
alfa2=0; beta2=0; gamma2=0;

matriz_pieza1=Desplazamiento(posicion(1), posicion(2), posicion(3))*Rotacionz(alfa)*Rotaciony(beta)*Rotacionx(gamma);
matriz_pieza2=Desplazamiento(posicion2(1), posicion2(2), posicion2(3))*Rotacionz(alfa2)*Rotaciony(beta2)*Rotacionx(gamma2);


pinta_bloque(matriz_pieza1,'b')
pinta_bloque(matriz_pieza2,'g')

G_T_pieza1=matriz_pieza1;
G_T_pieza2=matriz_pieza2;


% agarre

pieza_T_pinza=Desplazamiento(0,0,4)*Rotacionz(pi/2)*Rotaciony(pi);

G_T_pinza1=G_T_pieza1*pieza_T_pinza;
G_T_pinza2=G_T_pieza2*pieza_T_pinza;

matriz_pinza1=G_T_pinza1;
matriz_pinza2=G_T_pinza2;

q=[0 -1.5700 -1.5700 -1.5700 1.5700 0];
funcion_pinta_UR3_new(q, matriz_pinza2);
funcion_pinta_UR3_new(q, matriz_pinza1);




%%%%%%%%%%%%%%%%%%%%%%%%%
% configuraco de p. inv cinematico

codo=1;
avance=1;
simetrico=0;


 %pinza1
 [q1 q2 q3 q4 q5 q6]=inv_kinema_ur3_new(matriz_pinza1,codo,avance,simetrico); 

 funcion_pinta_UR3_new([q1 q2 q3 q4 q5 q6],matriz_pinza1);

 
%pinza2

[q1 q2 q3 q4 q5 q6]=inv_kinema_ur3_new(matriz_pinza2,codo,avance,simetrico); 

 funcion_pinta_UR3_new([q1 q2 q3 q4 q5 q6],matriz_pinza2);

 % aproximacion


 matriz_aprox=matriz_pinza1*Desplazamiento(0,0,-6);
 matriz_aprox2=matriz_pinza2*Desplazamiento(0,0,-6);


   %pinza1
 [q1 q2 q3 q4 q5 q6]=inv_kinema_ur3_new(matriz_aprox,codo,avance,simetrico); 

 funcion_pinta_UR3_new([q1 q2 q3 q4 q5 q6],matriz_aprox);

 
%pinza2

[q1 q2 q3 q4 q5 q6]=inv_kinema_ur3_new(matriz_aprox2,codo,avance,simetrico); 

 funcion_pinta_UR3_new([q1 q2 q3 q4 q5 q6],matriz_aprox2);
