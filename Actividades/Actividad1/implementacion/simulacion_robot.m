
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulación del movimiento de un robot móvil
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
clc


global l
global radio_rueda


l=1.0; %distancia entre rudas delanteras y traseras, tambien definido en modelo
radio_rueda=1;


mapa=[];

%Condiciones iniciales 
pose0=[0; 0; 0; 0; 0];

%final de la simulación
tf=10;

%paso de integracion
h=0.1;

%indice de la matriz
k=0;
%punto=[30 30];
%inicialización de variables
 pose(:,k+1)=pose0;
%valores para actualizazación
 tiempo=0;
 Integral=0; % valor de la integral en el instante 0
 error_ant=0;

 % otras constantes
 Delay=2;
 Periodo=6;
 Amplitud=pi/2;

pause(0.2)

while tiempo < tf 
    %actualización
    k=k+1;
    %t(k)=toc(tstart);
    t(k)=tiempo+h;
    dt=t(k)-tiempo;
    tiempo=t(k);
    
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Se definien las acciones de control
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    


%---------------------------------
% Aquí se calculan las referencias
%---------------------------------  
    referencia(k)=90;
    referencia(k) = 200 * t(k);
    referencia(k) = signal_vf_v2(t(k),Periodo,Delay,Amplitud);

 %-------------------------------
 % Aquí debe ir el controlador
 %--------------------------------

 
  error(k)=referencia(k)-pose(4,k);  % pose(4,k) es el angulo girado del motor en el instante k


  %% CONTROL PROPORCIONAL

   kp=0.6984; % constante del controlador proporcional

   Potencia_Cabeza(k)=kp*error(k); % señal la rampa
    
    
%% -----------------------
%% Control PI
%% -------------------
 Ganancia_p = 1.0226;
 Ganancia_I = 0.290;

Integral = Integral+error(k)*dt;
error_ant=error(k);
 
 Potencia_Cabeza(k) = Ganancia_p*error(k) + Ganancia_I*Integral; %controlador


%%PID
Ganancia_p = 1.0226;
Ganancia_I = 0.290;
Ganancia_D=0.15076;
Integral = Integral+error(k)*dt;
Derivativo = (error(k)-error_ant)/dt;
error_ant=error(k);
Potencia_Cabeza(k) = Ganancia_p*error(k) + Ganancia_I*Integral+Ganancia_D*Derivativo; %controlador

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Aquí se definen los valores de los actuadores
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if Potencia_Cabeza(k)>100
         Potencia_Cabeza(k)=100;
    elseif Potencia_Cabeza(k)<-100
        Potencia_Cabeza(k)=-100;
    end            
    Potencia_rueda_Derecha=0;
    Potencia_rueda_Izquierda=0;         
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
   if t(k)>5
       Perturbacion= -10;
   else 
       Perturbacion = 0;
   end
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
 
   conduccion=[Potencia_rueda_Derecha Potencia_rueda_Izquierda, Potencia_Cabeza(k),Perturbacion];
  
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%Se integra el modelo del robot diferencia para simularlo
    pose(:,k+1)=Robot_sim(t(k),pose(:,k),h,conduccion);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Representación del robot
   distancia=5; %proporcionada por el sonar
   mapa=pinta_robot_v3(pose(1,k+1),pose(2,k+1),pose(3,k+1),pose(4,k+1)*pi/180,distancia,mapa);
   grid on
   drawnow
   pause(0.05)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

%--------------------------
%Borra el último valor simulado para que la dimensión de pose coincida
%conla de t
pose(:,k+1)=[];
%--------------------------------


figure 
subplot(2,1,1), plot(t,pose(4,:)), hold on
plot(t,referencia), grid on

subplot(2,1,2), plot(t,error), hold on
plot(t,error,'*r'), grid on

