%Funci�n para calcular el likelikhood de la medicion de cada part�cula
%es llamada por robotics.ParticleFilter.correct()
%Todos los likelihoods me dan 0
function likelihood=measurement_model(particle_filter,predicted_particles,measurement,varargin)
    SIGMA_MODEL=0.8;
    ANGLE_LENGTH=10; %Me quedo con s�lo estos angulos
    DOWNSCALE_FACTOR=int32(length(measurement)/ANGLE_LENGTH);
    map=varargin{1};
    
    display("Calcule el Likelihood de la medici�n")
    angles=linspace(-pi/2,pi/2,ANGLE_LENGTH); %Esto esta hardcodeado
    measurement=downsample(measurement,DOWNSCALE_FACTOR); %Sub muestreo las mediciones
    max_range=varargin{2};
    likelihood=ones(particle_filter.NumParticles,1);
    measurement(isnan(measurement))=max_range;
    for row =1:particle_filter.NumParticles
        particle_position=predicted_particles(row,:);
        if getOccupancy(map,particle_position(1:2))<0.5
            ray_intercept_point=rayIntersection(map,particle_position,angles,max_range);
            particle_measurement=abs(ray_intercept_point-particle_position(1:2));
            particle_measurement(isnan(particle_measurement))=max_range;
            for index = 1:ANGLE_LENGTH 
                if ~isnan(measurement(index)) && ~isnan(particle_measurement(index)) 
                    likelihood(row)=likelihood(row)*(normpdf(measurement(index),particle_measurement(index),SIGMA_MODEL));
                elseif ~(isnan(measurement(index)) && isnan(measurement(index))) 
                    %Si uno es NaN y el otro no bajo el likelihood
                    likelihood(row)=likelihood(row)*0.0000001; %Bajo likelihood
                else
                    likelihood(row)=likelihood(row)*0.00001;
            
                end
            end
        else
            likelihood(row)=0;
        end
        
    end
end