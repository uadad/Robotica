clear all
clc

codigo=35;
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
    MoveL_Robot_lab([q1 q2 q3 q4 q5 q6], 0.5, 0.5,Identificador, codigo )

    %pick

    [q1 q2 q3 q4 q5 q6] = inv_kinema_ur3_new(G_T_Pinza, codo, avance, simetrico);
    MoveL_Robot_lab([q1 q2 q3 q4 q5 q6], 0.5, 0.5,Identificador, codigo)

    RG2_lab(25,Identificador, codigo)
    %despege pick 

    [q1 q2 q3 q4 q5 q6] = inv_kinema_ur3_new(Desplazamiento(0,0,5)*G_T_Pinza, codo, avance, simetrico);
    MoveJ_Robot_lab([q1 q2 q3 q4 q5 q6], 0.5, 0.5,Identificador, codigo)


    %aproximacion place

    [q1 q2 q3 q4 q5 q6] = inv_kinema_ur3_new(Desplazamiento(0,0,6*i)*G_T_Pinza1, codo, avance, simetrico);
    MoveL_Robot_lab([q1 q2 q3 q4 q5 q6], 0.5, 0.5,Identificador, codigo)


    %place
    [q1 q2 q3 q4 q5 q6] = inv_kinema_ur3_new(G_T_Pinza1*Desplazamiento(0,0,(-6)*(i-1)), codo, avance, simetrico);
    MoveL_Robot_lab([q1 q2 q3 q4 q5 q6], 0.5, 0.5,Identificador, codigo)

    RG2_lab(120,Identificador, codigo)
    %despege place

    [q1 q2 q3 q4 q5 q6] = inv_kinema_ur3_new(G_T_Pinza*Desplazamiento(0,0,(-6)*(i)), codo, avance, simetrico);
    MoveJ_Robot_lab([q1 q2 q3 q4 q5 q6], 0.5, 0.5,Identificador, codigo)
    
    %actualizacion
    matriz_pieza = matriz_pieza*Desplazamiento(-10,0,0);
end

pause(1)

for i=numero_piezas:-1:1
   
    matriz_pieza = matriz_pieza*Desplazamiento(10,0,0);
    
    G_T_Pinza = matriz_pieza*matriz_agarre;
    G_T_Pinza1 = matriz_pieza1*matriz_agarre;
    %aprox pick
    [q1 q2 q3 q4 q5 q6] = inv_kinema_ur3_new(G_T_Pinza*Desplazamiento(0,0,(-6)*(i)), codo, avance, simetrico);
    MoveJ_Robot_lab([q1 q2 q3 q4 q5 q6], 0.5, 0.5,Identificador, codigo)
    
    % pick
     [q1 q2 q3 q4 q5 q6] = inv_kinema_ur3_new(G_T_Pinza1*Desplazamiento(0,0,(-6)*(i-1)), codo, avance, simetrico);
    MoveL_Robot_lab([q1 q2 q3 q4 q5 q6], 0.5, 0.5,Identificador, codigo)

    RG2_lab(25,Identificador, codigo)
    
    % despegue pick
    [q1 q2 q3 q4 q5 q6] = inv_kinema_ur3_new(Desplazamiento(0,0,6*i)*G_T_Pinza1, codo, avance, simetrico);
    MoveL_Robot_lab([q1 q2 q3 q4 q5 q6], 0.5, 0.5,Identificador, codigo)
    
    % aprox place
    [q1 q2 q3 q4 q5 q6] = inv_kinema_ur3_new(Desplazamiento(0,0,5)*G_T_Pinza, codo, avance, simetrico);
    MoveJ_Robot_lab([q1 q2 q3 q4 q5 q6], 0.5, 0.5,Identificador, codigo)
    
    % place 
    [q1 q2 q3 q4 q5 q6] = inv_kinema_ur3_new(G_T_Pinza, codo, avance, simetrico);
    MoveL_Robot_lab([q1 q2 q3 q4 q5 q6], 0.5, 0.5,Identificador, codigo)

    RG2_lab(120,Identificador, codigo)
    
    % despegue place 
     [q1 q2 q3 q4 q5 q6] = inv_kinema_ur3_new(G_T_Pinza*Desplazamiento(0,0,-5), codo, avance, simetrico);
    MoveL_Robot_lab([q1 q2 q3 q4 q5 q6], 0.5, 0.5,Identificador, codigo )
    
    
    
end






