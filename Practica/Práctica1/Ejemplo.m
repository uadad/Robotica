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

%definición de la velocidad y error iniciales

error_i=0;

velocidad=10;

while  readTouch(Pulsador)==0
    
   distancia = readDistance(Sonar);
   disp(distancia);
   
%--------------------------  
%Controlador discreto  
%------------
  
   if distancia<0.20
       velocidad=-20;
   elseif distancia>0.40
       velocidad=20;
   end


%--------------------------
% Controlador PI
%-------------------------
%   error=(distancia-0.20);
%   error_i=error_i+error;
%   
%   velocidad=int8(350*error+0.70*error_i); 
%   
%   if velocidad >60
%       velocidad=60
%   elseif velocidad <-60
%       velocidad=-60
%   end
%    

%-------------------------------
% enviamos señal a los motores
%----------------------------

  motor_rueda_B.Speed=int8(velocidad);
  motor_rueda_C.Speed=int8(velocidad);

end
    
  
  motor_rueda_B.Speed=int8(0);
  motor_rueda_C.Speed=int8(0);