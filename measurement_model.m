%Funci�n para calcular el likelikhood de la medicion de cada part�cula
%es llamada por robotics.ParticleFilter.correct()
%Todos los likelihoods me dan 0
function likelihood=measurement_model(particle_filter,predicted_particles,measurement,varargin)
    SIGMA_MODEL=0.5;
    DOWNSAMPLE_FACTOR=40;
    map=varargin{1};
    
    display("Calcule el Likelihood de la medici�n")
    angle_length=int32(length(measurement)/DOWNSAMPLE_FACTOR);
    angles=linspace(-pi/2,pi/2,angle_length); %Esto esta hardcodeado
    max_range=varargin{2};
    likelihood=ones(particle_filter.NumParticles,1);
    for row =1:particle_filter.NumParticles
        particle_position=predicted_particles(row,:);
        ray_intercept_point=rayIntersection(map,particle_position,angles,max_range);
        particle_measurement=abs(ray_intercept_point-particle_position(1:2));
        if getOccupancy(map,particle_position(1:2))>0
            likelihood(row)=0;
        else    
            for index = 1:length(angles) 
                if ~isnan(measurement(index*DOWNSAMPLE_FACTOR)) && ~isnan(particle_measurement(index)) %Considero que cuando son NaN el likelihood es 1  

                    likelihood(row)=1;%likelihood(row)*(normpdf(measurement(index*DOWNSAMPLE_FACTOR),particle_measurement(index),SIGMA_MODEL));

                %elseif isnan(measurement(index)) && isnan(measurement(index))
                    %likelihood(row)=likelihood(row)*0.1;
                %else 
                    %likelihood(row)=likelihood(row)^2; %Reduzco el likelihood
                end
            end
        end
        likelihood=likelihood;
    end
end