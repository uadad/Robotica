function punto = pure_presuit(camino,pose,look_ahead)

 ind=funcion_minima_distancia(camino,[pose(1) pose(2)]);
 if(ind + look_ahead >length(camino))
     %punto=camino(mod(ind+look_ahead,length(camino)),:);
     punto=camino(length(camino),:);
 else   
     punto=camino(ind+look_ahead,:);
 end
end

