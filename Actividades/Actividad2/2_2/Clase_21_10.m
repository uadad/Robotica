

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


% agarre 1 por arriba situando la pinza alineada
% pieza_T_pinza=Desplazamiento(0,0,4)*Rotacionz(pi/2)*Rotaciony(pi);
 
% agarre 2 por arriba con el eje de la pinza invertido
% pieza_T_pinza=Desplazamiento(0,0,4)*Rotacionx(pi);   

% agarre 3 por el lado 
pieza_T_pinza=Desplazamiento(0,-0.8,4)*Rotacionx(-pi/2)*Rotacionz(pi/2);

G_T_pinza1_pick=G_T_pieza1*pieza_T_pinza;
G_T_pinza2_place=G_T_pieza2*pieza_T_pinza;

matriz_pinza1_pick=G_T_pinza1_pick;
matriz_pinza2_place=G_T_pinza2_place;

q=[0 -1.5700 -1.5700 -1.5700 1.5700 0];
%funcion_pinta_UR3_new(q, matriz_pinza2_place);
%funcion_pinta_UR3_new(q, matriz_pinza1_pick);




%%%%%%%%%%%%%%%%%%%%%%%%%
% configuraco de p. inv cinematico

codo=1;
avance=1;
simetrico=0;

 %Paso 1 ---aproxiamcion pick  con respecto al eje del sistema local x eso se multiplica dsp ----------------

 matriz_aprox=matriz_pinza1_pick*Desplazamiento(0,0,-6);

 [q1 q2 q3 q4 q5 q6]=inv_kinema_ur3_new(matriz_aprox,codo,avance,simetrico); 

 funcion_pinta_UR3_new([q1 q2 q3 q4 q5 q6],matriz_aprox);

 pause
 cla

 %------ Paso 2 pick -----
pinta_bloque(matriz_pieza1,'b')
pinta_bloque(matriz_pieza2,'g')
 
 [q1 q2 q3 q4 q5 q6]=inv_kinema_ur3_new(matriz_pinza1_pick,codo,avance,simetrico); 

 funcion_pinta_UR3_new([q1 q2 q3 q4 q5 q6],matriz_pinza1_pick);

pause
cla

pinta_bloque(matriz_pieza1,'b')
pinta_bloque(matriz_pieza2,'g')

% Movimento del despegue del pick con respecto al eje de referencia global 
 matriz_despegue_pick = Desplazamiento(0,0,6)*matriz_pinza1_pick;
 [q1 q2 q3 q4 q5 q6]=inv_kinema_ur3_new(matriz_despegue_pick,codo,avance,simetrico); 
 funcion_pinta_UR3_new([q1 q2 q3 q4 q5 q6],matriz_despegue_pick);

 pause 
 cla
 %%% Aproximacion del place 
pinta_bloque(matriz_pieza1,'b')
pinta_bloque(matriz_pieza2,'g')

 matriz_aprox2=Desplazamiento(0,0,6)*matriz_pinza2_place;
 [q1 q2 q3 q4 q5 q6]=inv_kinema_ur3_new(matriz_aprox2,codo,avance,simetrico); 

 funcion_pinta_UR3_new([q1 q2 q3 q4 q5 q6],matriz_aprox2);
 

pause
cla

% place 
pinta_bloque(matriz_pieza1,'b')
pinta_bloque(matriz_pieza2,'g')

 [q1 q2 q3 q4 q5 q6]=inv_kinema_ur3_new(matriz_pinza2_place,codo,avance,simetrico); 

 funcion_pinta_UR3_new([q1 q2 q3 q4 q5 q6],matriz_pinza2_place);

pause 
cla

pinta_bloque(matriz_pieza1,'b')
pinta_bloque(matriz_pieza2,'g')
 % despegue place 


  matriz_despegue_place = matriz_pinza1_pick*Desplazamiento(0,0,-6);
 [q1 q2 q3 q4 q5 q6]=inv_kinema_ur3_new(matriz_despegue_place,codo,avance,simetrico); 
 funcion_pinta_UR3_new([q1 q2 q3 q4 q5 q6],matriz_despegue_place);
