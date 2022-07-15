%Funci�n para actualizar la odometr�a de las particulas en el filtro
%es llamada por robotics.ParticleFilter.predict()

function predictParticles=movement_model(particle_filter,prev_particles,varargin)
    %Utiliza el modelo de movimiento, necesita recibir como argumentos
    %variables las velocodiades y el delta de tiempo.
    %movement_model(pf,prevParticles,velocidad_lineal,velocidad_angular,delta_tiempo)
    
    ALPHA_1=0.1;
    ALPHA_2=0.4;
    ALPHA_3=0.5;
    ALPHA_4=0.9;
    ALPHA_5=5;
    ALPHA_6=6;
    v=varargin{1};
    w=varargin{2};
    time_step=varargin{3};
    display("Use modelo de movimiento")
    predictParticles=zeros(size(prev_particles));
    for row =1:particle_filter.NumParticles %Si la velocidad angular es 0 tiene chances de que se enoje
        v_=v+normrnd(0,ALPHA_1*abs(v)+ALPHA_2*abs(w));
        w_=w+normrnd(0,ALPHA_3*abs(v)+ALPHA_4*abs(w));
        s_=0+normrnd(0,ALPHA_5*abs(v)+ALPHA_6*abs(w));
        theta=prev_particles(row,3);
        predictParticles(row,:)=prev_particles(row,:)+[v_/w_*(sin(theta+w_*time_step)-sin(theta)),...
                                                         v_/w_*(cos(theta)-cos(theta+w_*time_step)),...
                                                         (w_+s_)*time_step];
    
    end
    
end