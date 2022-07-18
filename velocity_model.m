function [vel,theta_end] = velocity_model(xy,theta,sample_time)
    vel = zeros(2,size(xy,1)-1);
    for i = 2:size(xy,1)
        j=i-1;
        
        dif_x = xy(j,1)-xy(i,1);
        dif_y = xy(j,2)-xy(i,2);
        
        mu = (1/2) * ( ( (dif_x)*cos(theta) + (dif_y)*sin(theta) ) / ((dif_y)*cos(theta) - (dif_x)*sin(theta)) );
        xstar = (xy(j,1) + xy(i,1))/2 + mu*(dif_y);
        ystar = (xy(j,2)+ xy(i,2))/2 + mu*(dif_y);
        rstar = sqrt((dif_x)^2 + (dif_y)^2 );
        deltatheta = atan2( xy(i,2) - ystar , xy(j,1) - xstar ) - atan2( xy(j,2) - ystar , xy(j,1) - xstar );
        theta_end = theta + deltatheta;
        vel(2,j) = deltatheta/sample_time; %w
        vel(1,j) = rstar * vel(2,j); %v     
    end
end
