function weight = measurement_model(z, x, l)
    % Computes the observation likelihood of all particles.
    %
    % The employed sensor model is range only.
    %
    % z: set of landmark observations. Each observation contains the id of the landmark observed in z(i).id and the measured range in z(i).range.
    % x: set of current particles
    % l: map of the environment composed of all landmarks
    sigma = [0.2];
    
    weight = zeros(size(x, 1),1);

    if size(z, 2) == 0
        return
    end
    weight=get_weight(z,x,l,sigma);
    

    weight = weight %./ size(z, 2);
end
function weight=get_weight(z,x,l,sigma)
    %Calcula el vector de pesos 
    landmark_number=size(z,2);
    particle_number=size(x,1);
    weight=zeros(particle_number,1);
    for i =1:landmark_number
        landmark_position = [l(z(i).id).x, l(z(i).id).y];
        for j = 1:particle_number
            weight(j)=prob_measurement_given_position(z(i),x(j,:),landmark_position,sigma);
        end
    end        
end
function prob= prob_measurement_given_position(z,x,landmark_pos,sigma)
%calculo P(Z|X) dada la posición del hito, el desvío y la posición actual
    d_=sqrt((landmark_pos(1)-x(1))^2+(landmark_pos(2)-x(2))^2);
    
    %alfa_=atan2((landmark_pos(2)-x(2)),landmark_pos(1)-x(1))-x(3);
    %alfa_meas=atan2(landmark_pos(1),landmark_pos(2))
    p_1=normpdf(z.range-d_,0,sigma);
    p_2=1;%normpdf(alfa_meas-angle_,0.01);
    prob=p_1*p_2;
end