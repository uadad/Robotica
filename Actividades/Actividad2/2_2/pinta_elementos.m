
%configuración de la pieza
posicion=[0 -10 0];
alfa=0; beta=0; gamma=pi/4;
  
matriz_pieza=Desplazamiento(posicion(1), posicion(2), posicion(3))*Rotacionz(alfa)*Rotaciony(beta)*Rotacionx(gamma);

%pinta_pieza_delgada(matriz_pieza)
pinta_bloque(matriz_pieza,'b')



 
 

