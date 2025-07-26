clear all
clc

codigo=1;
Identificador = Iniciacion('RBDK',codigo);

%configuración de la pieza
posicion=[40 -10 0];
alfa=0; beta=0; gamma=0;
  
matriz_pieza=Desplazamiento(posicion(1), posicion(2), posicion(3))*Rotacionz(alfa)*Rotaciony(beta)*Rotacionx(gamma);


%configuración de la pieza
posicion=[20 20 0];
  
matriz_pieza1=Desplazamiento(posicion(1), posicion(2), posicion(3))*Rotacionz(alfa)*Rotaciony(beta)*Rotacionx(gamma);



%agarres

%matriz_agarre = Desplazamiento(0,0,4)*Rotacionz(pi/2)*Rotaciony(pi); %Agarre 1
matriz_agarre = Desplazamiento(0,0,4)*Rotacionx(pi); %Agarre 2
%matriz_agarre = Desplazamiento(0,-0.8,4)*Rotacionx(-pi/2)*Rotacionz(pi/2); %Agarre 3

G_T_Pinza = matriz_pieza*matriz_agarre;
G_T_Pinza1 = matriz_pieza1*matriz_agarre;

q=[0 -1.5700 -1.5700 -1.5700 1.5700 0];

%conf p. inv cinematico

codo = 1;
avance = 1;
simetrico = 0;

numero_piezas=3;
for i=1:numero_piezas
    
    G_T_Pinza = matriz_pieza*matriz_agarre;
    G_T_Pinza1 = matriz_pieza1*matriz_agarre;

    %aproximacion pick

    [q1 q2 q3 q4 q5 q6] = inv_kinema_ur3_new(G_T_Pinza*Desplazamiento(0,0,-5), codo, avance, simetrico);
    MoveJ_Robot_lab([q1 q2 q3 q4 q5 q6], 1, 1,Identificador, codigo )

    %pick

    [q1 q2 q3 q4 q5 q6] = inv_kinema_ur3_new(G_T_Pinza, codo, avance, simetrico);
    MoveJ_Robot_lab([q1 q2 q3 q4 q5 q6], 1, 1,Identificador, codigo)

    RG2_lab(50,Identificador, codigo)
    %despege pick 

    [q1 q2 q3 q4 q5 q6] = inv_kinema_ur3_new(Desplazamiento(0,0,5)*G_T_Pinza, codo, avance, simetrico);
    MoveJ_Robot_lab([q1 q2 q3 q4 q5 q6], 1, 1,Identificador, codigo)


    %aproximacion place

    [q1 q2 q3 q4 q5 q6] = inv_kinema_ur3_new(Desplazamiento(0,0,6*i)*G_T_Pinza1, codo, avance, simetrico);
    MoveJ_Robot_lab([q1 q2 q3 q4 q5 q6], 1, 1,Identificador, codigo)


    %place
    [q1 q2 q3 q4 q5 q6] = inv_kinema_ur3_new(G_T_Pinza1*Desplazamiento(0,0,(-6)*(i-1)), codo, avance, simetrico);
    MoveJ_Robot_lab([q1 q2 q3 q4 q5 q6], 1, 1,Identificador, codigo)

    RG2_lab(120,Identificador, codigo)
    %despege place

    [q1 q2 q3 q4 q5 q6] = inv_kinema_ur3_new(G_T_Pinza*Desplazamiento(0,0,(-6)*(i)), codo, avance, simetrico);
    MoveJ_Robot_lab([q1 q2 q3 q4 q5 q6], 1, 1,Identificador, codigo)
    
    %actualizacion
    matriz_pieza = matriz_pieza*Desplazamiento(-10,0,0);
end





%lineal para detras

% [q1 q2 q3 q4 q5 q6] = inv_kinema_ur3_new(Desplazamiento(6*i,0,0)*G_T_Pinza1, codo, avance, simetrico);
    MoveJ_Robot_lab([q1 q2 q3 q4 q5 q6], 1, 1,Identificador, codigo)


    %place
    [q1 q2 q3 q4 q5 q6] = inv_kinema_ur3_new(G_T_Pinza1*Desplazamiento((-6)*(i-1),0,0), codo, avance, simetrico);
    MoveJ_Robot_lab([q1 q2 q3 q4 q5 q6], 1, 1,Identificador, codigo)

    RG2_lab(120,Identificador, codigo)
    %despege place

    [q1 q2 q3 q4 q5 q6] = inv_kinema_ur3_new(G_T_Pinza*Desplazamiento((-6)*(i),0,0), codo, avance, simetrico);
    MoveJ_Robot_lab([q1 q2 q3 q4 q5 q6], 1, 1,Identificador, codigo)
    
    %actualizacion
    matriz_pieza = matriz_pieza*Desplazamiento(-10,0,0);

%




