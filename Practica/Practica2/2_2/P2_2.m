clear all
clc

codigo=20;
Identificador = Iniciacion('Robot_1',codigo);

%configuración de la pieza
posicion=[20 -10 0];
alfa=0; beta=0; gamma=-(pi/4);
  
matriz_caja=Desplazamiento(posicion(1), posicion(2), posicion(3))*Rotacionz(alfa)*Rotaciony(beta)*Rotacionz(0);


%configuración de la pieza
posicion=[24 5 0];

matriz_pieza1=Desplazamiento(posicion(1), posicion(2), posicion(3))*Rotacionz(0)*Rotaciony(0)*Rotacionz(gamma);



%agarres

matriz_agarre = Desplazamiento(0,0,4)*Rotacionz(pi/2)*Rotaciony(pi); %Agarre 1
%matriz_agarre = Desplazamiento(0,0,4)*Rotacionx(pi); %Agarre 2
%matriz_agarre = Desplazamiento(0,-0.8,4)*Rotacionx(-pi/2)*Rotacionz(pi/2); %Agarre 3

G_T_caja = matriz_caja*matriz_agarre;
G_T_place = matriz_pieza1*matriz_agarre;

q=[0 -1.5700 -1.5700 -1.5700 1.5700 0];


%conf p. inv cinematico

codo = 1;
avance = 1;
simetrico = 0;

numero_piezas=6;

RG2_lab(50,Identificador, codigo)

num_piezas_fila=3;
for i=1:numero_piezas
    
    G_T_Pinza = matriz_caja*matriz_agarre;
    G_T_Pinza1 = matriz_pieza1*matriz_agarre;

    %aproximacion pick

    [q1 q2 q3 q4 q5 q6] = inv_kinema_ur3_new(G_T_Pinza*Desplazamiento(0,0,-5), codo, avance, simetrico);
    MoveL_Robot_lab([q1 q2 q3 q4 q5 q6], 0.5, 0.5,Identificador, codigo )

    %pick

    [q1 q2 q3 q4 q5 q6] = inv_kinema_ur3_new(G_T_Pinza, codo, avance, simetrico);
    MoveL_Robot_lab([q1 q2 q3 q4 q5 q6], 0.5, 0.5,Identificador, codigo)

    RG2_lab(30,Identificador, codigo)
    %despege pick 

    [q1 q2 q3 q4 q5 q6] = inv_kinema_ur3_new(Desplazamiento(0,0,5)*G_T_Pinza, codo, avance, simetrico);
    MoveJ_Robot_lab([q1 q2 q3 q4 q5 q6], 0.5, 0.5,Identificador, codigo)


    %aproximacion place
  
 
    [q1 q2 q3 q4 q5 q6] = inv_kinema_ur3_new(Desplazamiento(0,0,7)*G_T_Pinza1, codo, avance, simetrico);
    MoveL_Robot_lab([q1 q2 q3 q4 q5 q6], 0.5, 0.5,Identificador, codigo)


    %place
    [q1 q2 q3 q4 q5 q6] = inv_kinema_ur3_new(G_T_Pinza1, codo, avance, simetrico);
    MoveL_Robot_lab([q1 q2 q3 q4 q5 q6], 0.5, 0.5,Identificador, codigo)

    RG2_lab(50,Identificador, codigo)
    %despege place

    [q1 q2 q3 q4 q5 q6] = inv_kinema_ur3_new(G_T_Pinza1*Desplazamiento(0,0,-7), codo, avance, simetrico);
    MoveJ_Robot_lab([q1 q2 q3 q4 q5 q6], 0.5, 0.5,Identificador, codigo)
    
    %actualizacion
    if mod(i,3)==0
       matriz_caja=matriz_caja*Desplazamiento(-20,-5,0);
       matriz_pieza1=matriz_pieza1*Desplazamiento(5,-9.7,0);
    else
         matriz_caja=matriz_caja*Desplazamiento(10,0,0);
       matriz_pieza1=matriz_pieza1*Desplazamiento(0,4.85,0);
    end
end



