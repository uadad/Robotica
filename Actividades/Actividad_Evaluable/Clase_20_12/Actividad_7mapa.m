global l
global radio_rueda
global camino
global pose
global punto

l=3.5; %distancia entre rudas delanteras y traseras, tambien definido en modelo
radio_rueda=1;

%Condiciones iniciales 
%pose0=[10; 15; -pi/4]; posef=[80; 80; -pi/4]; sol2
% pose0=[0;0;-pi/4]; posef=[80; 80; -pi/4]; % sol1
%pose0=[0;0;-pi/4]; posef=[80; 80; pi];    % sol_3

%-------------------------------------------
% Carga y representación del mapa
%

MAPA = imread('cuadro6.bmp');

%Transformación para colocar correctamente el origen del Sistema de
%Referencia
MAPA(1:end,:,:)=MAPA(end:-1:1,:,:);
image(MAPA);
axis xy
%---------------------------------------------------

dd=9*direccion;
da=dd;

posicion_despegue=[pose0(1)+(dd*cos(pose0(3))) pose0(2)+(dd*sin(pose0(3)))];

posicion_aterriza=[posef(1)-(da*cos(posef(3))) posef(2)-(da*sin(posef(3)))];

%definicion del poligono


%Llamada del algoritmo a*
delta=10;
Optimal_path=A_estrella_modificado_2(MAPA,delta,pose0,posef);

ds=0.1;
xc=[pose0(1) posicion_despegue(1) Optimal_path(:,1)' posicion_aterriza(1) posef(1)];
yc=[pose0(2) posicion_despegue(2) Optimal_path(:,2)' posicion_aterriza(2) posef(2)];
camino=funcion_spline_cubica_varios_puntos(xc,yc,ds)';

t0=0;
%final de la simulación
tf=35;
%paso de integracion
h=0.1;
%vector tiempo
t=0:h:tf;
%indice de la matriz
k=0;
%inicialización valores iniciales
pose(:,k+1)=pose0;
t(k+1)=t0;
look_ahead=30;
 
while (t0+h*k) < tf 
    %actualización
    k=k+1;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %valores de los parámetros de control
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    punto=pure_presuit(camino,pose(:,k),look_ahead);
    punto_final=camino(length(camino),:);
    [V p]=funcion_controlador_geometrico3(pose(:,k),punto,punto_final,direccion);
    [velocidad_derecha velocidad_izquierda]=funcion_modelo_cinematico_inverso(V,p);
    conduccion=[velocidad_derecha velocidad_izquierda];

    %metodo de integración ruge-kuta
    pose(:,k+1)=kuta_diferencial_mapa(t(k),pose(:,k),h,conduccion,MAPA);
end