%%  Primer script %%

clear all
close all
pause(0.1);
%Definicion del Robot
mi_Robot=legoev3('usb')

%Definicion de los motores
motor_rueda_B = motor(mi_Robot,'B')
motor_rueda_C = motor(mi_Robot,'C')

%Activación de los motores
start(motor_rueda_B);
start(motor_rueda_C);

%Definición de los sensores

Pulsador=touchSensor(mi_Robot,2); %pulsador
Sonar = sonicSensor(mi_Robot,4); %definición del sonar

N=10;
tiempo= zeros(1,N);
distancias= zeros(1,N);
t0=1;
for i=1:N
  tiempo(i)=t0;
  distancias(i)=readDistance(Sonar);
  pause(1);
  t0=t0+1;
end



%%  Segundo script %%

clear all
close all
%Definicion del Robot
mi_Robot=legoev3('USB')
%Definicion de los motores
motor_rueda_A = motor(mi_Robot,'A');
%Activación de los motores
start(motor_rueda_A);
%definición de la velocidad y error iniciales
error_i=0;
valor=10;
Pulsador=touchSensor(mi_Robot,2); %pulsador
while  readTouch(Pulsador)==0
  motor_rueda_A.Speed=int8(valor);
end  
  valor=0;
  motor_rueda_A.Speed=int8(valor);
  

  
%%  Pruebas script %%

clear all
close all
%Definicion del Robot
mi_Robot=legoev3('USB')
Pulsador=touchSensor(mi_Robot,2); %pulsador

%Definicion de los motores
motor_Cabeza= motor(mi_Robot,'A');
%Activación de los motores
start(motor_Cabeza);
resetRotation(motor_Cabeza);
while  readTouch(Pulsador)==0
  disp 'Esperando Pulsacion'
end
while  readTouch(Pulsador)==1
end
k=0;
clc;
tstart=tic;
while  readTouch(Pulsador)==0
    k=k+1;
    t(k)=toc(tstart);
    motor_Cabeza.Speed=50;
    Giro_Cabeza(k)=readRotation(motor_Cabeza);
    disp 'Pulse para salir'
end
motor_Cabeza.Speed=0;
stop(motor_Cabeza);
plot(t,Giro_Cabeza)




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Simulación del movimiento de un robot móvil
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
close all
%Definicion del Robot
mi_Robot=legoev3('USB')
Pulsador=touchSensor(mi_Robot,2); %pulsador

%Definicion de los motores
motor_Cabeza= motor(mi_Robot,'A');

%Activación de los motores
start(motor_Cabeza);

resetRotation(motor_Cabeza);
 

%indice de la matriz
k=0;
%valores para actualizazación
tstart=tic;
pause(0.2)
while  readTouch(Pulsador)==0
  disp 'Esperando Pulsacion'
end
while  readTouch(Pulsador)==1
end
while  readTouch(Pulsador)==0
    %actualización
    k=k+1;
    t(k)=toc(tstart);   
    referencia(k)=90;%200*t(k);
    Giro_Cabeza(k)=double(readRotation(motor_Cabeza));
    error(k)= referencia(k) - Giro_Cabeza(k);
 %-------------------------------
 % Control Proporcional
 %-------------------------------
    Ganancia_p = 0.69184;%pidtuner Matlab
    Potencia_Cabeza=int8(Ganancia_p*error(k));
    if Potencia_Cabeza>100
        Potencia_Cabeza=100;
    elseif Potencia_Cabeza<-100
        Potencia_Cabeza=-100;
    end
     motor_Cabeza.Speed=int8(Potencia_Cabeza);
     Giro_Cabeza(k)=readRotation(motor_Cabeza);
end
motor_Cabeza.Speed=0;
stop(motor_Cabeza);
plot(t,Giro_Cabeza(k),'b'),hold on
plot(t,error,'r'), hold on
plot(t,referencia,'g')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Simulación del movimiento de un robot móvil
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
close all
%Definicion del Robot
mi_Robot=legoev3('USB')

%Definicion de los motores
motor_Cabeza= motor(mi_Robot,'A');

%Activación de los motores
start(motor_Cabeza);

Pulsador=touchSensor(mi_Robot,2); %pulsador
sonar=sonicSensor(mi_Robot);

resetRotation(motor_Cabeza);
 

%indice de la matriz
k=0;
%valores para actualizazación
tstart=tic;
     

    delay=toc(tstart)+0.2;
    periodo=6;
    Amplitud=90

while  readTouch(Pulsador)==0
  disp 'Esperando Pulsacion'
end

while  readTouch(Pulsador)==1
end

while  readTouch(Pulsador)==0
    %actualización
    k=k+1;
    t(k)=toc(tstart);  
    Giro_Cabeza(k)=double(readRotation(motor_Cabeza));
    distancia(k)=readDistance(sonar); 
    referencia(k)=signal_vf_v2( t(k), periodo, delay, Amplitud ); 
    error(k)=referencia(k)-Giro_Cabeza(k);
 %-------------------------------
 % Control Proporcional
 %-------------------------------
    Ganancia_p = 0.69184;%pidtuner Matlab
    Potencia_Cabeza=int8(Ganancia_p*error(k));
    if Potencia_Cabeza>100
        Potencia_Cabeza=100;
    elseif Potencia_Cabeza<-100
        Potencia_Cabeza=-100;
    end
     motor_Cabeza.Speed=int8(Potencia_Cabeza);
     Giro_Cabeza(k)=readRotation(motor_Cabeza);
end
motor_Cabeza.Speed=0;
stop(motor_Cabeza);
plot(t,Giro_Cabeza,'b'),hold on
plot(t,error,'r'), hold on
plot(t,referencia,'g')