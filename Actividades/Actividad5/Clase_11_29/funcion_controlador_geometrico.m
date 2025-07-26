
function [V, p] = funcion_controlador_geometrico(pose, punto)
    delta=(pose(1)-punto(1))*sin(pose(3))-(pose(2)-punto(2))*cos(pose(3));
    L_h=sqrt(((pose(1)-punto(1))^2)+((pose(2)-punto(2))^2));
    distancia=sqrt(((pose(1)-punto(1))^2)+((pose(2)-punto(2))^2));
    p=2*delta/(L_h^2);
    kpv=1;
    V=kpv*distancia;
end

