
PARTICLES_NUM=100;
INIT_POS=[0,0,0];
POSITION_LIMITS=[0,1;0,1;pi,-pi];
PARTICLE_FILTER_RESAMPLING_INTERVAL=2;  %Remuestrea cada 2 actualizaciones de odometr�a
particle_filter=robotics.ParticleFilter; %Creo objeto filtro de part�culas
particle_filter.StateEstimationMethod='mean'; %Tomo el promedio de las part�culas como mi estado mas probable
particle_filter.StateTransitionFcn=@movement_model; %Funci�n para actualizar la odometr�a
particle_filter.MeasurementLikelihoodFcn=@measurement_model; %Funci�n del modelo de medici�n
particle_filter.ResamplingPolicy.SamplingInterval=PARTICLE_FILTER_RESAMPLING_INTERVAL;
particle_filter.ResamplingMethod='systematic'; % Remuestreo por SUS
initialize(particle_filter,PARTICLES_NUM,POSITION_LIMITS)

figure()
scatter(particle_filter.Particles(:,1),particle_filter.Particles(:,2)),hold on;
particle_filter.predict(1,0,1)
particle_filter.correct(map)