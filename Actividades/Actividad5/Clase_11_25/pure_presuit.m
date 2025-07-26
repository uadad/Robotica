function punto = pure_presuit(camino,pose,look_ahead)

 ind=funcion_minima_distancia(camino,[pose(1) pose(2)]);
 punto=camino(ind+look_ahead,:);
end

