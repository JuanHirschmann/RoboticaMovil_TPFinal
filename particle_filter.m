function particles=particle_filter(old_particles,movement)
    %Recibe las particulas del paso anterior. Actualiza
    %las particulas muestreando el modelo de movimiento
    % y calcula los pesos muestreando el modelo de medicion
    %Devuelve las particulas en el siguiente paso.
    particles = old_particles;

    

    %Resolver el muestreo
    %new_particles = sample_motion_model(data.timestep(t).odometry, particles)
    %weights = measurement_model(data.timestep(t).sensor, new_particles, landmarks);

    normalizer = sum(weights);
    weights = weights ./ normalizer;
    particles = resample(new_particles, weights);

    %current_position=mean_position(particles, weights);
end