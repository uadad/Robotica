function orden_minimo= funcion_minima_distancia(path, punto)  % indice del punto mas cercano

distancia=sqrt((punto(1)-path(:,1)).^2+(punto(2)-path(:,2)).^2);

[minimo orden_minimo]=min(sqrt(distancia));

end