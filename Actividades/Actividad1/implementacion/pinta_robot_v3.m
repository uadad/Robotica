function mapa_out=pinta_robot_v3(x,y,theta,alfa,distancia,mapa)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here


persistent robot cabeza

if isempty(robot)
    robot = hgtransform; %definida fuera
end

if isempty(cabeza)
    cabeza = hgtransform('Parent',robot);
end


R=rectangle('Position',[-1.5 -1.5 3 3],'Parent',robot);
R.LineWidth=2;
M = makehgtform('translate',[x y 0], 'zrotate',theta);
robot.Matrix=M;

rueda_derecha=hgtransform('Parent',robot);
rd=rectangle('Position',[-0.5 -0.1 1 0.2],'Parent',rueda_derecha);
M = makehgtform('translate',[0 1 0]);
rueda_derecha.Matrix=M;

rueda_izquierda=hgtransform('Parent',robot);
ri=rectangle('Position',[-0.5 -0.1 1 0.2],'Parent',rueda_izquierda);
M = makehgtform('translate',[0 -1 0]);
rueda_izquierda.Matrix=M;

%alngulo de la cabeza
%alfa=pi/4;

%cabeza=hgtransform('Parent',robot);%definida fuera
ca=rectangle('Position',[-0.25 -0.5 0.5 1],'Parent',cabeza);
M = makehgtform('translate',[1 0 0],'zrotate',alfa);
cabeza.Matrix=M;

Mt=robot.Matrix*cabeza.Matrix;

punto=Mt*[double(distancia) 0 0 1]';



%d=animatedline(punto(1),punto(2),'Marker','*');

%axis([-75 75 -75 75]);

axis([-10 10 -10 10]);

mapa_out=[mapa; punto(1) punto(2)];

d=animatedline(double(mapa_out(:,1)),double(mapa_out(:,2)),'Marker','*','LineStyle','none', 'Color',[0 0 1]);

end

