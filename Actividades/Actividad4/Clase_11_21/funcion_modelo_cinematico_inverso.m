function [velocidad_derecha, velocidad_izquierda] = funcion_modelo_cinematico_inverso(V, p)
global l
global radio_rueda

velocidad_derecha=V*(1+l*p)/radio_rueda;
velocidad_izquierda=V*(1-l*p)/radio_rueda;

end

