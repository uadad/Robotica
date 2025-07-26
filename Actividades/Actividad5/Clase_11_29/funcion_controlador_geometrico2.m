function [V, p] = funcion_controlador_geometrico2(pose, punto,punto_final)  % punto_final del camino
    delta=(pose(1)-punto(1))*sin(pose(3))-(pose(2)-punto(2))*cos(pose(3));
    L_h=sqrt(((pose(1)-punto(1))^2)+((pose(2)-punto(2))^2));
    distancia=sqrt(((punto_final(1)-pose(1))^2)+((punto_final(2)-pose(2))^2));
    p=2*delta/(L_h^2)
    kpv=1;
    V=kpv*distancia;
    if V>30
        V=30;
    end
    if V<0.01
        V=0;
    end
end

