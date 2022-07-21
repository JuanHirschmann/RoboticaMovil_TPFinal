function sample=random_empty_point(map,x_lims,y_lims)
%Elige del mapa un punto inicial para el robot en una parte no ocupada
    occupied = true;
    theta = pi/2+(pi)*rand();
    while occupied==true
        x = x_lims(1)+(x_lims(2)-x_lims(1))*rand();
        y = y_lims(1)+(y_lims(2)-y_lims(1))*rand();
        occupied = getOccupancy(map,[x,y])> map.FreeThreshold;
    end
    sample = [x,y,theta];
end